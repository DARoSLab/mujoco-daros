import casadi as ca
import pinocchio.casadi as pin
import pinocchio
import numpy as np
from typing import List, Dict
import os
import time
import yaml
import sys


class Utils:

    @staticmethod
    def loadPinocchioModel(urdf_path):
        ncmodel = pinocchio.buildModelFromUrdf(urdf_path, pinocchio.JointModelFreeFlyer())
        model = pin.Model(ncmodel)
        return model
    
    @staticmethod
    def create_inverse_dynamics_function(model):
        q = ca.SX.sym("q", model.nq)
        v = ca.SX.sym("v", model.nv)
        a = ca.SX.sym("a", model.nv)

        pin_data = model.createData()
        tau = pin.rnea(model, pin_data, q, v, a)

        return ca.Function("ID", [q, v, a], [tau])

    @staticmethod
    def create_contact_forward_kinematics_function(model, contact_link_names):
        q = ca.SX.sym("q", model.nq)
        contact_points = ca.SX.zeros(3 * len(contact_link_names))

        pin_data = model.createData()
        pin.forwardKinematics(model, pin_data, q)
        pin.updateFramePlacements(model, pin_data)

        for i, link_name in enumerate(contact_link_names):
            link_id = model.getFrameId(link_name)
            link_pos = pin_data.oMf[link_id].translation
            contact_points[3*i:3*i+3] = link_pos

        return ca.Function("contact_forward_kinematics", [q], [contact_points])

    @staticmethod
    def create_contact_velocity_function(model, contact_link_names):
        q = ca.SX.sym("q", model.nq)
        v = ca.SX.sym("v", model.nv)
        contact_vels = ca.SX.zeros(3 * len(contact_link_names))

        pin_data = model.createData()
        pin.forwardKinematics(model, pin_data, q, v)
        pin.updateFramePlacements(model, pin_data)

        for i, link_name in enumerate(contact_link_names):
            link_id = model.getFrameId(link_name)
            link_vel_world = pin.getFrameVelocity(model, pin_data, link_id, pin.ReferenceFrame.WORLD)
            contact_vels[3*i:3*i+3] = link_vel_world.vector[:3]

        return ca.Function("contact_velocity", [q, v], [contact_vels])

    @staticmethod
    def create_contact_jacobian_function(model, contact_link_names):
        q = ca.SX.sym("q", model.nq)
        n_contacts = len(contact_link_names)
        Jc = ca.SX.zeros(3 * n_contacts, model.nv)

        pin_data = model.createData()
        pin.computeJointJacobians(model, pin_data, q)

        for i, link_name in enumerate(contact_link_names):
            link_id = model.getFrameId(link_name)
            J_link = pin.getFrameJacobian(model, pin_data, link_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            Jc[3*i:3*i+3, :] = J_link[:3, :]

        return ca.Function("contact_jacobian", [q], [Jc])

    @staticmethod
    def create_config_integration_function(model):
        q = ca.SX.sym("q", model.nq)
        v = ca.SX.sym("v", model.nv)
        dt = ca.SX.sym("dt", 1)

        q_next = pin.integrate(model, q, v * dt)

        return ca.Function("integrator", [q, v, dt], [q_next])

    @staticmethod
    def create_mapping_from_local_angular_velocity_to_euler_angles_xyz_derivative():
        euler_angles = ca.SX.sym("euler_angles", 3)
        x, y, z = euler_angles[0], euler_angles[1], euler_angles[2]

        sy, cy = ca.sin(y), ca.cos(y)
        sz, cz = ca.sin(z), ca.cos(z)
        cz_cy, sz_cy = cz / cy, sz / cy

        M = ca.SX.zeros(3, 3)
        M[0, 0], M[0, 1], M[0, 2] = cz_cy, -sz_cy, 0
        M[1, 0], M[1, 1], M[1, 2] = sz, cz, 0
        M[2, 0], M[2, 1], M[2, 2] = -sy * cz_cy, sy * sz_cy, 1

        return ca.Function("mapping", [euler_angles], [M])

    @staticmethod
    def mapping_from_local_angular_velocity_to_euler_angles_xyz_derivative(euler_angles):
        x, y, z = euler_angles[0], euler_angles[1], euler_angles[2]

        sy, cy = ca.sin(y), ca.cos(y)
        sz, cz = ca.sin(z), ca.cos(z)
        cz_cy, sz_cy = cz / cy, sz / cy

        M = ca.SX.zeros(3, 3)
        M[0, 0], M[0, 1], M[0, 2] = cz_cy, -sz_cy, 0
        M[1, 0], M[1, 1], M[1, 2] = sz, cz, 0
        M[2, 0], M[2, 1], M[2, 2] = -sy * cz_cy, sy * sz_cy, 1

        return M

    @staticmethod
    def mapping_from_global_angular_velocity_to_euler_angles_zyx_derivative(euler_angles, angular_velocity):
        x, y, z = euler_angles[0], euler_angles[1], euler_angles[2]
        wx, wy, wz = angular_velocity[0], angular_velocity[1], angular_velocity[2]

        sz, cz = ca.sin(z), ca.cos(z)
        sy, cy = ca.sin(y), ca.cos(y)
        tmp = cz * wx / cy + sz * wy / cy

        return ca.vertcat(tmp, -sz * wx + cz * wy, sy * tmp + wz)

    @staticmethod
    def create_rotation_matrix_from_xyz_euler_angles_function():
        euler_angles = ca.SX.sym("euler_angles", 3)
        x, y, z = euler_angles[0], euler_angles[1], euler_angles[2]

        cx, cy, cz = ca.cos(x), ca.cos(y), ca.cos(z)
        sx, sy, sz = ca.sin(x), ca.sin(y), ca.sin(z)

        rotation_matrix = ca.SX.zeros(3, 3)
        rotation_matrix[0, 0] = cy * cz
        rotation_matrix[0, 1] = -cy * sz
        rotation_matrix[0, 2] = sy
        rotation_matrix[1, 0] = cz * sx * sy + cx * sz
        rotation_matrix[1, 1] = cx * cz - sx * sy * sz
        rotation_matrix[1, 2] = -cy * sx
        rotation_matrix[2, 0] = sx * sz - cx * cz * sy
        rotation_matrix[2, 1] = cz * sx + cx * sy * sz
        rotation_matrix[2, 2] = cx * cy

        return ca.Function("rotation_matrix_XYZ", [euler_angles], [rotation_matrix])

    @staticmethod
    def create_quat_to_rpy_function():
        quat = ca.SX.sym("quat", 4)
        qx, qy, qz, qw = quat[0], quat[1], quat[2], quat[3]

        r = ca.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        p = ca.asin(2 * (qw * qy - qz * qx))
        y = ca.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        rpy = ca.vertcat(r, p, y)

        return ca.Function("quat_to_rpy", [quat], [rpy])

    @staticmethod
    def add_contact_constraint_flat(opti, ugrf_k, csk, mu):
        num_contacts = csk.shape[0]

        for foot in range(num_contacts):
            grf = ugrf_k[3*foot:3*foot+3]
            f_x, f_y, f_z = grf[0], grf[1], grf[2]
            csk_foot = csk[foot]

            opti.subject_to(csk_foot * f_z >= 0)
            opti.subject_to(csk_foot * f_x <= mu * csk_foot * f_z)
            opti.subject_to(csk_foot * f_y <= mu * csk_foot * f_z)
            opti.subject_to(csk_foot * f_x >= -mu * csk_foot * f_z)
            opti.subject_to(csk_foot * f_y >= -mu * csk_foot * f_z)

    @staticmethod
    def rpy_to_quat(rpy):
        r, p, y = rpy[0], rpy[1], rpy[2]

        cy, sy = ca.cos(y / 2), ca.sin(y / 2)
        cp, sp = ca.cos(p / 2), ca.sin(p / 2)
        cr, sr = ca.cos(r / 2), ca.sin(r / 2)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return ca.vertcat(qx, qy, qz, qw)

    @staticmethod
    def create_config_integration_function_rpy(nv):
        q = ca.SX.sym("q", nv)
        v = ca.SX.sym("v", nv)
        dt = ca.SX.sym("dt", 1)

        # q_next = ca.SX.zeros(nv)

        body_pos = q[:3]
        lin_vel_body = v[:3]

        body_ori_rpy = q[3:6]
        ang_vel = v[3:6]

        R = pin.rpy.rpyToMatrix(body_ori_rpy)

        euler_rate = ca.mtimes(Utils.mapping_from_local_angular_velocity_to_euler_angles_xyz_derivative(body_ori_rpy), ang_vel)
        # euler_rate = ca.mtimes(Utils.mapping_from_local_angular_velocity_to_euler_angles_xyz_derivative(body_ori_rpy), ca.mtimes(R, ang_vel))
        
        # euler_rate = Utils.mapping_from_global_angular_velocity_to_euler_angles_zyx_derivative(body_ori_rpy, ca.mtimes(R, ang_vel))

        # lin_vel_world = ca.mtimes(R, lin_vel_body)

        # q_next[:3] = body_pos + dt * lin_vel_world
        # q_next[3:6] = body_ori_rpy + dt * euler_rate
        # q_next[6:] = q[6:] + dt * v[6:]

        body_pos_next = body_pos + dt * ca.mtimes(R, lin_vel_body)
        body_ori_rpy_next = body_ori_rpy + dt * euler_rate
        qjt_next = q[6:] + dt * v[6:]

        q_next = ca.vertcat(body_pos_next, body_ori_rpy_next, qjt_next)

        return ca.Function("integrator", [q, v, dt], [q_next])


        
if __name__ == "__main__":
    config_path ="../configs/staccatoe.yaml"
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    urdf_path = config["urdf_path"]
    contact_link_names = config["contact_link_names"]

    ncmodel = pinocchio.buildModelFromUrdf(urdf_path)
    model = pin.Model(ncmodel)
    contact_forward_kinematics = Utils.create_contact_forward_kinematics_function(model, contact_link_names)
    contact_velocity = Utils.create_contact_velocity_function(model, contact_link_names)
    contact_jacobian = Utils.create_contact_jacobian_function(model, contact_link_names)

    q = np.zeros(model.nq)
    contact_points = contact_forward_kinematics(q)
    contact_vels = contact_velocity(q, np.zeros(model.nv))
    contact_jac = contact_jacobian(q)

