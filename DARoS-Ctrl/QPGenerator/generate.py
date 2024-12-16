import argparse
import yaml
import os
import time
import casadi as ca
import numpy as np
import pinocchio.casadi as pin
import pinocchio
from utils import Utils


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_path", help="Path to YAML config file" , default="../configs/staccatoe.yaml")
    parser.add_argument("--use_approx_hess",
                        help="Use approximate hessian", type=bool, default=True)
    args = parser.parse_args()

    with open(args.config_path, 'r') as file:
        config = yaml.safe_load(file)

    print("Generating functions for QP solver...")
    print("Use approx hessian: ", args.use_approx_hess)

    robot_name = config['name']
    urdf_path = config['urdf_path']
    contact_link_names = config['contact_link_names']
    horizon = config['horizon']

    print("===============Config Parameters===============")
    print(f"Robot name: {robot_name}")
    print(f"URDF path: {urdf_path}")
    print("Contact link names:")
    for link in contact_link_names:
        print(f"\t{link}")
    print(f"Horizon: {horizon}")
    print("===============================================")

    model = Utils.loadPinocchioModel(urdf_path)
    contact_fk_func = Utils.create_contact_forward_kinematics_function(
        model, contact_link_names)
    contact_jac_func = Utils.create_contact_jacobian_function(
        model, contact_link_names)
    contact_vel_func = Utils.create_contact_velocity_function(
        model, contact_link_names)
    id_func = Utils.create_inverse_dynamics_function(model)
    int_func = Utils.create_config_integration_function_rpy(model.nv)
    int_quat_func = Utils.create_config_integration_function(model)
    if config.get('use_motor_limits', False):
        joint_to_motor_func = ca.Function.load(config["joint_to_motor_func"])
        qjt_sym = ca.MX.sym("qjt", model.nv - 6)
        motor_sym = joint_to_motor_func(qjt_sym)
        joint_jac_func = ca.Function("joint_jac", [qjt_sym], [ca.jacobian(motor_sym, qjt_sym)]).expand()

    nq = model.nq - 1  # replace quat with rpy
    nv = model.nv
    nu = model.nv - 6
    n_contact = len(contact_link_names)

    opti = ca.Opti()

    # Parameters
    q_min = opti.parameter(nq)
    q_max = opti.parameter(nq)
    vmax = opti.parameter(nv)
    tau_max = opti.parameter(nu)
    mu = opti.parameter(1)
    dt = opti.parameter(1)
    base_vel_cmd = opti.parameter(6)
    base_height_cmd = opti.parameter(1)
    base_ori_rpy_cmd = opti.parameter(3)
    qjt_cmd = opti.parameter(nu)
    tau_0_prev = opti.parameter(nu)

    Q_tau = opti.parameter(nu)
    Q_fr = opti.parameter(3 * n_contact)
    Q_qjt = opti.parameter(nu)
    Q_vjt = opti.parameter(nu)
    Q_base_pos = opti.parameter(3)
    Q_base_ori = opti.parameter(3)
    Q_base_vel = opti.parameter(6)

    contact_schedule = opti.parameter(n_contact, horizon + 1)
    contact_height_ref = opti.parameter(n_contact, horizon + 1)

    q0 = opti.parameter(nq)
    v0 = opti.parameter(nv)

    # Variables
    q = opti.variable(nq, horizon)
    v = opti.variable(nv, horizon)
    tau = opti.variable(nu, horizon)
    fr = opti.variable(n_contact * 3, horizon)

    Wk = ca.diag(ca.vertcat(Q_qjt, Q_vjt, Q_tau, Q_fr,
                 Q_base_pos, Q_base_ori, Q_base_vel))

    nres = Wk.shape[0]
    residual = ca.MX.zeros(nres, horizon)

    # opti.subject_to(q[:, 0] == q0)
    # opti.subject_to(v[:, 0] == v0)

    # add q0 to q concatenation first column
    q_aug = ca.horzcat(q0, q)
    v_aug = ca.horzcat(v0, v)

    for k in range(horizon):
        qk = q_aug[:, k]
        vk = v_aug[:, k]
        qk_next = q_aug[:, k + 1]
        vk_next = v_aug[:, k + 1]
        # qk = q[:, k]
        # vk = v[:, k]
        # qk_next = q[:, k + 1]
        # vk_next = v[:, k + 1]

        tauk = tau[:, k]
        frk = fr[:, k]
        csk = contact_schedule[:, k]

        csk_rep = ca.reshape(ca.repmat(csk, 1, 3).T, 3 * n_contact, 1)

        vdotk = (vk_next - vk) / dt
        qk_jt = qk[6:]
        vk_jt = vk[6:]
        base_pos = qk[:3]
        base_ori_rpy = qk[3:6]
        base_vel = vk[:6]
        base_ori_quat = Utils.rpy_to_quat(base_ori_rpy)

        base_pos_next = qk_next[:3]
        base_ori_rpy_next = qk_next[3:6]
        base_vel_next = vk_next[:6]
        base_ori_quat_next = Utils.rpy_to_quat(base_ori_rpy_next)

        qk_pin = ca.vertcat(base_pos, base_ori_quat, qk_jt)
        qk_next_pin = ca.vertcat(base_pos_next, base_ori_quat_next, qk_jt)

        tau_id = id_func(qk_pin, vk, vdotk)
        Jc = contact_jac_func(qk_pin)
        JcT = Jc.T
        pf = contact_fk_func(qk_pin)
        vf = contact_vel_func(qk_pin, vk)

        pfz = pf[2::3]
        pfz_ref = contact_height_ref[:, k]

        qk_next_int = int_func(qk, 0.5 * (vk + vk_next), dt)

        # qk_next_int = int_func(qk, vk_next, dt) # implicit integration

        tau_calc = tau_id - ca.mtimes(JcT, frk)

        if k > 0:
            base_xy_ref = q0[:2] + k*base_vel_cmd[:2] * dt

            residual[:nu, k - 1] = qk_jt - qjt_cmd
            residual[nu:2*nu, k - 1] = vk_jt

            if k == 1:
                residual[2*nu:3*nu, k - 1] = tauk - tau_0_prev

            base_pos_cmd = ca.vertcat(base_xy_ref, base_height_cmd)
            residual[3*nu:3*nu+3*n_contact, k - 1] = frk
            residual[3*nu+3*n_contact:3*nu+3*n_contact +
                     3, k - 1] = base_pos - base_pos_cmd
            residual[3*nu+3*n_contact+3:3*nu+3*n_contact +
                     6, k - 1] = base_ori_rpy - base_ori_rpy_cmd
            residual[3*nu+3*n_contact+6:3*nu+3*n_contact +
                     12, k - 1] = base_vel - base_vel_cmd

            opti.subject_to(opti.bounded(-vmax, vk, vmax))
            opti.subject_to(opti.bounded(q_min, qk, q_max))

            opti.subject_to(csk_rep * vf == 0)

            opti.subject_to(pfz == pfz_ref)

        opti.subject_to(qk_next == qk_next_int)
        # opti.subject_to(qk_next_pin == int_quat_func(qk_pin, 0.5 * (vk + vk_next), dt))

        if config.get('use_motor_limits', False):
            jt_motor_jac = joint_jac_func(qk_jt)
            tau_jointk = ca.mtimes(jt_motor_jac.T, tauk)
            opti.subject_to(tau_jointk == tau_calc[6:])
            # print warning in yellow use_motor_limits is true
            if k == 0:
                print("\033[93mWarning: Using motor limits.\033[0m")
        else:
            opti.subject_to(tauk == tau_calc[6:])
            
        opti.subject_to(tau_calc[:6] == 0)
        opti.subject_to(opti.bounded(-tau_max, tauk, tau_max))
        opti.subject_to((1 - csk_rep) * frk == 0)

        Utils.add_contact_constraint_flat(opti, frk, csk, mu)

    qN = q_aug[:, horizon]
    vN = v_aug[:, horizon]

    # qN = q[:, horizon]
    # vN = v[:, horizon]

    qN_jt = qN[6:]
    vN_jt = vN[6:]
    base_posN = qN[:3]
    base_ori_rpyN = qN[3:6]
    base_velN = vN[:6]
    base_ori_quatN = Utils.rpy_to_quat(base_ori_rpyN)
    qN_pin = ca.vertcat(base_posN, base_ori_quatN, qN_jt)

    pfN = contact_fk_func(qN_pin)
    pfNz = pfN[2::3]
    vfN = contact_vel_func(qN_pin, vN)
    csN = contact_schedule[:, horizon]
    csN_rep = ca.reshape(ca.repmat(csN, 1, 3).T, 3 * n_contact, 1)
    pfzN_ref = contact_height_ref[:, horizon]

    opti.subject_to(opti.bounded(q_min, qN, q_max))
    opti.subject_to(opti.bounded(-vmax, vN, vmax))
    opti.subject_to(csN_rep*vfN == 0)
    opti.subject_to(pfNz == pfzN_ref)

    base_xy_ref = q0[:2] + horizon*base_vel_cmd[:2] * dt

    base_pos_cmdN = ca.vertcat(base_xy_ref, base_height_cmd)

    # Wk = ca.diag(ca.vertcat(Q_qjt, Q_vjt, Q_tau, Q_fr, Q_base_pos, Q_base_ori, Q_base_vel))

    residual[:nu, horizon - 1] = qN_jt - qjt_cmd
    residual[nu:2*nu, horizon - 1] = vN_jt
    residual[2*nu:3*nu, horizon - 1] = ca.MX.zeros(nu, 1)
    residual[3*nu:3*nu+3*n_contact, horizon -
             1] = ca.MX.zeros(3 * n_contact, 1)
    residual[3*nu+3*n_contact:3*nu+3*n_contact+3,
             horizon - 1] = base_posN - base_pos_cmdN
    residual[3*nu+3*n_contact+3:3*nu+3*n_contact+6,
             horizon - 1] = base_ori_rpyN - base_ori_rpy_cmd
    residual[3*nu+3*n_contact+6:3*nu+3*n_contact +
             12, horizon - 1] = base_velN - base_vel_cmd

    res_flat = ca.reshape(residual, nres * horizon, 1)
    I = ca.MX.eye(horizon)
    W = ca.kron(I, Wk)

    cost = 0.5 * ca.mtimes(ca.mtimes(res_flat.T, W), res_flat)

    opti.minimize(cost)

    C = opti.f
    G = opti.g
    lb = opti.lbg
    ub = opti.ubg

    decision_vars = [ca.vec(q), ca.vec(v), ca.vec(tau), ca.vec(fr)]
    z = ca.vertcat(*decision_vars)

    params = [Q_qjt, Q_vjt, Q_tau, Q_fr, Q_base_pos, Q_base_ori, Q_base_vel,
              base_height_cmd, base_ori_rpy_cmd, base_vel_cmd, dt, mu,
              q_min, q_max, vmax, tau_max, ca.vec(contact_schedule),
              ca.vec(contact_height_ref), q0, v0, qjt_cmd, tau_0_prev]
    p = ca.vertcat(*params)

    residual_jac = ca.jacobian(res_flat, z)

    if args.use_approx_hess:
        print("\033[93mWarning: Using Approx hessian.\033[0m")
        Hess = ca.mtimes(residual_jac.T, ca.mtimes(
            W, residual_jac))  # Gauss-Newton approximation
    else:
        print(
            "\033[93mWarning: Using numerical hessian computation. This may be slow.\033[0m")
        Hess = ca.hessian(cost, z)[0]

    Hess_uptri = ca.project(ca.triu(Hess), Hess.sparsity())

    robot_results_path = config.get(
        'results_path', os.path.join('..', 'results', robot_name))
    os.makedirs(robot_results_path, exist_ok=True)

    real_type = config.get('casadi_real', 'double')
    int_type = config.get('casadi_int', 'long long int')

    func_opts = {
        'with_header': True,
        'casadi_real': real_type,
        'casadi_int': int_type
    }

    filename = f"{robot_name}_horizon_{horizon}_real_{real_type}_qp_funcs"

    print(f"Number of decision variables: {z.shape[0]}")
    print(f"Number of constraints: {G.shape[0]}")
    print("Generating functions...")

    grad_C_z = ca.gradient(C, z)

    opts_jac = {
        'helper_options': {
            'verbose': False,
            'ad_weight': 1
        }
    }
    jac_G_z = ca.jacobian(G, z, opts_jac)

    jac_G_z_func = ca.Function("jac_G_z", [z, p], [jac_G_z]).expand()

    jac_G_z_func.save(f"{filename}_jac_G_z.casadi")

    qp_funcs = ca.Function(
        filename, [z, p], [grad_C_z, Hess_uptri, jac_G_z, lb - G, ub - G]).expand()

    start_time = time.time()
    qp_funcs.generate(filename, func_opts)

    # move generated header file to result folder

    qp_funcs.save(f"{filename}.casadi")
    os.rename(f"{filename}.h", os.path.join(
        robot_results_path, f"{filename}.h"))

    compiled_lib_path = config["compiled_lib_path"]

    compile_command = f"gcc -fPIC -shared -O3 {filename}.c -o {compiled_lib_path}/{filename}.so"

    os.system(compile_command)

    # remove .c file
    os.remove(f"{filename}.c")

    end_time = time.time()
    print(f"Function generation time: {(end_time - start_time) / 60:.2f} min")

    start_time = time.time()
    filename = f"{robot_name}_horizon_{horizon}_real_{real_type}_linesearch_funcs"

    l_viol = ca.fmax(0.0, lb - G)
    u_viol = ca.fmax(0.0, G - ub)
    constr_viol = ca.sum1(ca.power(l_viol, 2) + ca.power(u_viol, 2)) * dt

    linesearch_funcs = ca.Function(
        filename, [z, p], [C*dt, constr_viol, grad_C_z]).expand()

    print("Generating Line search functions...")

    linesearch_funcs.generate(filename, func_opts)

    # move generated header file to result folder

    os.rename(f"{filename}.h", os.path.join(
        robot_results_path, f"{filename}.h"))

    compile_command = f"gcc -fPIC -shared -O3 {filename}.c -o {compiled_lib_path}/{filename}.so"

    os.system(compile_command)

    os.remove(f"{filename}.c")
    end_time = time.time()
    print(
        f"Linear search Function generation time: {(end_time - start_time) / 60:.2f} min")


if __name__ == "__main__":
    main()
