#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "NMPC.hpp"
// #include <casadi/casadi.hpp>
#include <chrono>
#include <cstring>
#include <Configuration.h>
#include <Timer.h>

void NMPC::lineSearch()
{
    OSQPFloat alpha = alpha_max;
    OSQPFloat theta_k;
    OSQPFloat phi_k;
    OSQPFloat theta_next;
    OSQPFloat phi_next;

    auto lin_res = linesearch_funcs(casadi::DMVector{z_dm, p_dm});
    phi_k = (OSQPFloat)lin_res[0].scalar();   // cost
    theta_k = (OSQPFloat)lin_res[1].scalar(); // constraint violation

    // static int counter = 0;
    // std::cout << "====================" << std::endl;
    // std::cout << "z: dm: " << z_dm(casadi::Slice(0, 6)) << std::endl;
    // // std::cout << "p: dm: " << p_dm(casadi::Slice(119, 119+12)) << std::endl;
    // std::cout << "cost: " << phi_k << " constraint violation: " << theta_k << std::endl;

    // if (counter++ > 5)
    // {

    //     exit(0);
    // }
    casadi::DM grad_phi_z = lin_res[2];
    casadi::DM qp_sol = casadi::DM::zeros(nvar, 1);
    // Get a pointer to the internal data of the CasADI DM
    double *qp_sol_ptr = qp_sol.ptr();
    memcpy(qp_sol_ptr, solver->solution->x, nvar * sizeof(OSQPFloat));


    // std::cout << "qp_sol: " << qp_sol << std::endl;

    OSQPFloat grad_phi_dot_delta_w;
    grad_phi_dot_delta_w = (OSQPFloat)casadi::DM::dot(grad_phi_z, qp_sol).scalar();

    bool accepted = false;

    step_length = alpha;
    init_cost = phi_k;
    init_constraint_violation = theta_k;
    final_cost = phi_k;
    final_constraint_violation = theta_k;
    acceptance_type = -1;

    while (!accepted && alpha >= alpha_min)
    {
        casadi::DM z_next = z_dm + alpha * qp_sol;
        lin_res = linesearch_funcs(casadi::DMVector{z_next, p_dm});
        phi_next = (OSQPFloat)lin_res[0].scalar();   // cost
        theta_next = (OSQPFloat)lin_res[1].scalar(); // constraint violation
        
        if (theta_next > theta_max)
        {
            if (theta_next < (1 - gamma_theta) * theta_k) 
            {
                accepted = true;
                acceptance_type = 0;
            }
        }
        else if (std::max(theta_next, theta_k) < theta_min)   // constraint violation is small
        {

            if (grad_phi_dot_delta_w < 0 && phi_next < phi_k + eta * alpha * grad_phi_dot_delta_w) // Armijo condition
            {
                accepted = true;
                acceptance_type = 1;
            }
        }
        else if (phi_next < phi_k - gamma_phi * theta_k || theta_next < (1.0 - gamma_theta) * theta_k)
        {
            accepted = true;
            acceptance_type = 2;
        }
        // std::cout<<"phi_next: "<<phi_next<<" theta_next: "<<theta_next<<" alpha: "<<alpha<<std::endl;
        // std::cout<<"phi_k: "<<phi_k<<" theta_k: "<<theta_k<<std::endl;
        if (!accepted)
        {
            alpha *= gamma_alpha;

        }
        theta_k = theta_next;
        phi_k = phi_next;
    }
    final_cost = phi_k;
    final_constraint_violation = theta_k;

    step_length = alpha;
    // std::cout<<"step_length: "<<step_length<<std::endl;
    // std::cout<< "cost: "<< lin_res[0].scalar() << " constraint violation: " << lin_res[1].scalar() << "acceptance_type "<<acceptance_type << std::endl;
}
void NMPC::initNMPC()
{

    osqp_set_default_settings(settings);

    settings->alpha = osqp_alpha;
    settings->max_iter = osqp_max_iter;
    settings->eps_abs = osqp_eps_abs;
    settings->eps_rel = osqp_eps_rel;
    settings->warm_starting = osqp_warm_start;
    settings->eps_prim_inf = osqp_eps_prim_inf;
    settings->eps_dual_inf = osqp_eps_dual_inf;
    settings->verbose = osqp_verbose;
    settings->rho = osqp_rho;
    settings->adaptive_rho = osqp_adaptive_rho;
    settings->adaptive_rho_interval = osqp_adaptive_rho_interval;
    settings->check_termination = osqp_check_termination;
    settings->scaling = osqp_scaling;
    settings->sigma = osqp_sigma;
    qp_funcs = casadi::external("qpfunc", "./nlmpc.so");
    linesearch_funcs = casadi::external("linesearch", "./linesearch.so");
    auto z_sp = qp_funcs.sparsity_in(0);
    auto p_sp = qp_funcs.sparsity_in(1);
    auto l_sp = qp_funcs.sparsity_out(3);
    auto P_sp = qp_funcs.sparsity_out(1);
    auto A_sp = qp_funcs.sparsity_out(2);

    nvar = z_sp.size1();
    nparam = p_sp.size1();
    ncon = l_sp.size1();
    nnz_P = P_sp.nnz();
    nnz_A = A_sp.nnz();

    printf("========NMPC========\n");
    printf("nvar: %d\n", (int)nvar);
    printf("nparam: %d\n", (int)nparam);
    printf("ncon: %d\n", (int)ncon);
    printf("nnz_P: %d\n", (int)nnz_P);
    printf("nnz_A: %d\n", (int)nnz_A);
    printf("====================\n");

    P_i.resize(nnz_P);
    P_p.resize(nvar + 1);
    P_x.resize(nnz_P);

    P_i = P_sp.get_row();
    P_p = P_sp.get_col();

    A_i.resize(nnz_A);
    A_p.resize(ncon + 1);
    A_x.resize(nnz_A);

    l.resize(ncon);
    u.resize(ncon);
    q.resize(nvar);

    P_i = casadi::vector_static_cast<OSQPInt>(P_sp.get_row());
    P_p = casadi::vector_static_cast<OSQPInt>(P_sp.get_colind());

    A_i = casadi::vector_static_cast<OSQPInt>(A_sp.get_row());
    A_p = casadi::vector_static_cast<OSQPInt>(A_sp.get_colind());

    P_updated_idx.clear();
    A_updated_idx.clear();
    for (OSQPInt i = 0; i < (int)P_x.size(); ++i)
    {
        P_updated_idx.push_back(i);
    }
    for (OSQPInt i = 0; i < (int)A_x.size(); ++i)
    {
        A_updated_idx.push_back(i);
    }

    z_dm = casadi::DM::zeros(nvar);
    p_dm = casadi::DM::zeros(nparam);

    res.resize(5);

    csc_set_data(&A, ncon, nvar, A_x.size(), A_x.data(), A_i.data(), A_p.data());
    csc_set_data(&P, nvar, nvar, P_x.size(), P_x.data(), P_i.data(), P_p.data());
    OSQPInt exitflag = osqp_setup(&solver, &P, q.data(), &A, l.data(), u.data(), ncon, nvar, settings);
    if (exitflag != 0)
    {
        std::cout << "Failed to setup OSQP solver. Exitflag: " << exitflag << std::endl;
        exit(-1);
    }

}

void NMPC::updateQPMatrices(const std::vector<OSQPFloat> &p)
{
    std::copy(z.begin(), z.end(), z_dm.ptr());
    std::copy(p.begin(), p.end(), p_dm.ptr());

    res = qp_funcs(casadi::DMVector{z_dm, p_dm});

    const casadi::DM &grad_C = res[0];
    const casadi::DM &Hess_uptri = res[1];
    const casadi::DM &jac_G = res[2];
    const casadi::DM &lb = res[3];
    const casadi::DM &ub = res[4];

    // std::cout <<"Hess_uptri: \n" << Hess_uptri << std::endl;
    // std::cout <<"jac_G: \n" << jac_G << std::endl;

    P_x.clear();
    A_x.clear();
    q.clear();
    l.clear();
    u.clear();

    P_x.insert(P_x.end(), Hess_uptri.nonzeros().begin(), Hess_uptri.nonzeros().end());
    A_x.insert(A_x.end(), jac_G.nonzeros().begin(), jac_G.nonzeros().end());
    q.insert(q.end(), grad_C.nonzeros().begin(), grad_C.nonzeros().end());
    l.insert(l.end(), lb.nonzeros().begin(), lb.nonzeros().end());
    u.insert(u.end(), ub.nonzeros().begin(), ub.nonzeros().end());
    // Update OSQP problem
    osqp_update_data_mat(solver,
                        P_x.data(), P_updated_idx.data(), P_updated_idx.size(),
                        A_x.data(), A_updated_idx.data(), A_updated_idx.size());

    osqp_update_data_vec(solver, q.data(), l.data(), u.data());

}
void NMPC::solve(const std::vector<OSQPFloat> &p, int num_iter)
{
    for (int iter = 0; iter < num_iter; ++iter)
    {
        Timer timer0;
        updateQPMatrices(p);
        // std::cout<<"[NLMPC] updating time: "<<timer0.getMs()<<std::endl;
        // Solve updated problem
        Timer timer1;
        auto exitflag = osqp_solve(solver);
        // std::cout<<"[NLMPC] solver time: "<<timer1.getMs()<<std::endl;
        std::lock_guard<std::mutex> lock(solution_mutex);
        Timer timer2;
        lineSearch();
        // std::cout<<"[NLMPC] line search time: "<<timer2.getMs()<<std::endl;
        if (exitflag == 0)
        {
            for (OSQPInt i = 0; i < nvar; ++i)
            {
                z[i] += step_length * solver->solution->x[i];
            }
        }
        else
        {
            std::cout << "Failed to solve QP. Exitflag: " << exitflag << std::endl;
        }
    }

    sol_timestamp = std::chrono::high_resolution_clock::now();

}


void NMPC::loadNMPCConfigParams(const std::string &file)
{
  ParamHandler handler(file);

  if (!handler.fileOpenedSuccessfully())
  {
    throw std::runtime_error("Could not load the parameter file: " + file);
  }

  handler.getValue<OSQPFloat>("alpha_fixed", alpha_fixed);
  handler.getValue<OSQPFloat>("alpha_min", alpha_min);
  handler.getValue<OSQPFloat>("alpha_max", alpha_max);
  handler.getValue<OSQPFloat>("theta_max", theta_max);
  handler.getValue<OSQPFloat>("theta_min", theta_min);
  handler.getValue<OSQPFloat>("eta", eta);
  handler.getValue<OSQPFloat>("gamma_phi", gamma_phi);
  handler.getValue<OSQPFloat>("gamma_theta", gamma_theta);
  handler.getValue<OSQPFloat>("gamma_alpha", gamma_alpha);
  handler.getValue<bool>("use_fixed_alpha", use_fixed_alpha);

  std::cout << "use_fixed_alpha: " << use_fixed_alpha << std::endl;
  handler.getValue<OSQPFloat>("osqp_alpha", osqp_alpha);
  handler.getValue<OSQPFloat>("osqp_eps_abs", osqp_eps_abs);
  handler.getValue<OSQPFloat>("osqp_eps_rel", osqp_eps_rel);
  handler.getValue<OSQPFloat>("osqp_eps_prim_inf", osqp_eps_prim_inf);
  handler.getValue<OSQPFloat>("osqp_eps_dual_inf", osqp_eps_dual_inf);
  handler.getValue<OSQPFloat>("osqp_rho", osqp_rho);
  handler.getValue<OSQPFloat>("osqp_sigma", osqp_sigma);
  handler.getValue<OSQPFloat>("osqp_scaling", osqp_scaling);

  handler.getValue<OSQPInt>("osqp_max_iter", osqp_max_iter);
  handler.getValue<OSQPInt>("osqp_adaptive_rho_interval", osqp_adaptive_rho_interval);
  handler.getValue<OSQPInt>("osqp_check_termination", osqp_check_termination);

  handler.getValue<bool>("osqp_adaptive_rho", osqp_adaptive_rho);
  handler.getValue<bool>("osqp_warm_start", osqp_warm_start);
  handler.getValue<bool>("osqp_verbose", osqp_verbose);

  printf("[NMPC] Parameter Setup is completed\n");
}