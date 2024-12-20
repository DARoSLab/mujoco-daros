#pragma once
#ifndef NLP_HELPER_HPP
#define NLP_HELPER_HPP
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "osqp.h"
#include <casadi/casadi.hpp>
#include <mutex>
#include <ParamHandler/ParamHandler.hpp>
// #define PROFILE_SOLVER

// include the correct header file based on the configuration
#ifndef NMPC_HORIZON
#define NMPC_HORIZON 10
#endif


#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define CONCAT(a, b, c, d, e, f) a##b##c##d##e##f
#define INCLUDE_FILE(robot, horizon, real) TOSTRING(CONCAT(robot, _horizon_, horizon, _real_, real, _qp_funcs.h))
#define QP_FUNC_NAME(robot, horizon, real) CONCAT(robot, _horizon_, horizon, _real_, real, _qp_funcs)
#define QP_FUNC_SO_PATH(robot, horizon, real) THIS_COM "Systems/" #robot "/" #robot "_config/" TOSTRING(QP_FUNC_NAME(robot, horizon, real)) ".so"

#define LINESEARCH_INCLUDE_FILE(robot, horizon, real) TOSTRING(CONCAT(robot, _horizon_, horizon, _real_, real, _linesearch_funcs.h))
#define LINESEARCH_FUNC_NAME(robot, horizon, real) CONCAT(robot, _horizon_, horizon, _real_, real, _linesearch_funcs)
#define LINESEARCH_FUNC_SO_PATH(robot, horizon, real) THIS_COM "Systems/" #robot "/" #robot "_config/" TOSTRING(LINESEARCH_FUNC_NAME(robot, horizon, real)) ".so"

struct NMPCSolution
{
    std::vector<OSQPFloat> z;
    std::chrono::system_clock::time_point timestamp;
    OSQPFloat step_length = 1.0;
    OSQPFloat init_cost = 0.0;
    OSQPFloat init_constraint_violation = 0.0;
    OSQPFloat final_cost = 0.0;
    OSQPFloat final_constraint_violation = 0.0;
    OSQPInt acceptance_type = -1; // 0: case 1, 1: case 2, 2: case 3 -1: not accepted
};
class NMPC
{
public:
    NMPC()
    {
    }
    ~NMPC()
    {

        osqp_cleanup(solver);
        if (settings)
        {
            free(settings);
        }
    }

    void initNMPC();

    void solve(const std::vector<OSQPFloat> &p, int num_iter);

    NMPCSolution getSolution() const
    {
        std::lock_guard<std::mutex> lock(solution_mutex);
        return {z, sol_timestamp, step_length, init_cost, init_constraint_violation, final_cost, final_constraint_violation, acceptance_type};
    }

    void setInitialGuess(const std::vector<OSQPFloat> &z0)
    {
        this->z = z0;
    }

    void setLineSearchParameters(bool use_fixed_alpha_, OSQPFloat alpha_fixed_, OSQPFloat alpha_min_, OSQPFloat alpha_max_, OSQPFloat theta_max_, OSQPFloat theta_min_, OSQPFloat eta_, OSQPFloat gamma_phi_, OSQPFloat gamma_theta_, OSQPFloat gamma_alpha_)
    {
        this->use_fixed_alpha = use_fixed_alpha_;
        this->alpha_min = alpha_min_;
        this->alpha_max = alpha_max_;
        this->theta_max = theta_max_;
        this->theta_min = theta_min_;
        this->eta = eta_;
        this->gamma_phi = gamma_phi_;
        this->gamma_theta = gamma_theta_;
        this->gamma_alpha = gamma_alpha_;
        this->alpha_fixed = alpha_fixed_;
        step_length = alpha_max_;
    }

    void setOSQPSettings(OSQPFloat alpha, OSQPFloat eps_abs, OSQPFloat eps_rel, OSQPFloat eps_prim_inf, OSQPFloat eps_dual_inf, OSQPFloat rho, OSQPFloat sigma, OSQPFloat scaling, OSQPInt max_iter, OSQPInt adaptive_rho_interval, OSQPInt check_termination, bool adaptive_rho, bool warm_start, bool verbose)
    {
        this->osqp_alpha = alpha;
        this->osqp_eps_abs = eps_abs;
        this->osqp_eps_rel = eps_rel;
        this->osqp_eps_prim_inf = eps_prim_inf;
        this->osqp_eps_dual_inf = eps_dual_inf;
        this->osqp_rho = rho;
        this->osqp_sigma = sigma;
        this->osqp_scaling = scaling;
        this->osqp_max_iter = max_iter;
        this->osqp_adaptive_rho_interval = adaptive_rho_interval;
        this->osqp_check_termination = check_termination;
        this->osqp_adaptive_rho = adaptive_rho;
        this->osqp_warm_start = warm_start;
        this->osqp_verbose = verbose;
    }

    int getHorizon() const
    {
        return horizon;
    }

private:
    std::vector<OSQPFloat> z;
    OSQPCscMatrix P, A;
    std::vector<OSQPInt> P_i, P_p, A_i, A_p;
    std::vector<OSQPFloat> P_x, A_x, q, l, u;
    std::vector<OSQPInt> P_updated_idx, A_updated_idx;
    casadi::DM z_dm;
    casadi::DM p_dm;
    std::vector<casadi::DM> res;

    std::chrono::system_clock::time_point sol_timestamp;

    OSQPSolver *solver = nullptr;
    OSQPSettings *settings = (OSQPSettings *)malloc(sizeof(OSQPSettings));
    casadi::Function qp_funcs;
    casadi::Function linesearch_funcs;

    mutable std::mutex solution_mutex;

    OSQPInt nvar, ncon, nparam, nnz_P, nnz_A;
    int horizon = NMPC_HORIZON;

    // Line search parameters
    bool use_fixed_alpha = true;

    OSQPFloat step_length = 1.0;
    OSQPFloat init_cost = 0.0;
    OSQPFloat init_constraint_violation = 0.0;
    OSQPFloat final_cost = 0.0;
    OSQPFloat final_constraint_violation = 0.0;
    OSQPInt acceptance_type = -1; // 0: case 1, 1: case 2, 2: case 3 -1: not accepted

    OSQPFloat alpha_fixed = 1e-3;
    OSQPFloat alpha_min = 1e-4;
    OSQPFloat alpha_max = 1;
    OSQPFloat theta_max = 1e-2;
    OSQPFloat theta_min = 1e-6;
    OSQPFloat eta = 1e-4;
    OSQPFloat gamma_phi = 1e-6;
    OSQPFloat gamma_theta = 1e-6;
    OSQPFloat gamma_alpha = 0.5;

    // OSQP settings
    OSQPFloat osqp_alpha = 1.4;
    OSQPFloat osqp_eps_abs = 1e-3;
    OSQPFloat osqp_eps_rel = 1e-3;
    OSQPFloat osqp_eps_prim_inf = 1e-4;
    OSQPFloat osqp_eps_dual_inf = 1e-4;
    OSQPFloat osqp_rho = 2e-2;
    OSQPFloat osqp_sigma = 1e-6;
    OSQPFloat osqp_scaling = 2.0;

    OSQPInt osqp_max_iter = 100;
    OSQPInt osqp_adaptive_rho_interval = 19;
    OSQPInt osqp_check_termination = 20;

    bool osqp_adaptive_rho = false;
    bool osqp_warm_start = true;
    bool osqp_verbose = false;

    static constexpr size_t PROFILE_WINDOW = 1000;
    std::vector<double> update_qp_times;
    std::vector<double> solver_times;


    void lineSearch();
    void updateQPMatrices(const std::vector<OSQPFloat> &p);
    void loadNMPCConfigParams(const std::string &file);
};
#endif 