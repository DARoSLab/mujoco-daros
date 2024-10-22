/*================ Regularized Predictive Controller ==================*/
/*
 * Implementation of the MIT Cheetah predictive locomotion controller.
 * The Regularized Predictive Controller (RPC) determines optimal
 * predicted input vectors from the robot's COM to each foot in stance,
 * r_hat_f, and associated ground reaction forces, f_hat_f, based
 * on a given state trajectory to follow.
 *
 * @author  Gerardo Bledt
 * @date    February 26, 2017
 * @version 0.5
 *
 * TO DO:
 *  x Gait independent framework...
 *      will have to re do this entire file basically...
 *  x Fix the hessian calculation for the final timestep (seems correct now...)
 *  x Re-do the MATLAB portion for final constraint, constraint jacobian,
 *      and hessian because contact_state_{k+1} is no longer relevant then
 *  x MATLAB constraint jacobian needs to be transposed and then will need
 *      to revert iRow <-> jCol here
 *  x Get rid of Eigen stuff, no longer necessary
 *  x Initialize function takes forever... with generalized gait and no
 *      Eigen... it should be much faster
 *  x Add LCM sending out of results (just passed them back instead)
 *  x Recieve LCM for the gait parameters, current robot state, and
 *      some sort of desired trajectory... may need to adapt it here
 *  x Interface with the simulation... hopefully not too difficult
 *  - Overall clean up
 *  - Figure out whats going on with the gradient in the derivative test
 *  x Min timestep numerical bug (dt > 1e-3) at least
 *  x Bounds corresponding to actual footstep kinematics
 *  x Yaw bug of instability when > pi/2 ish
 *  x "Invisible Wall" bug
 *      x Both were dumb mistake... subtracted COM from foot positions twice
 *  - Tune gains correctly
 *  x Allow for enabling or disabling legs (for < 4 legged gaits)
 *  x MATLAB send in environment parameters
 *
 *  - LCM TO DOs:
 *      x Get current state
 *      x Get current foot positions
 *      x Get the gait parameters
 *      x Enabled leg parameter
 *      x Get RPC gain parameter tuning
 *      x Get Robot Parameters
 *      x Send out resulting forces
 *      x Send out time vector divisions (dt_pred)
 *      x MATLAB side LCM listener
 *
 *  - Ideas:
 *      x Add the full rotation matrix to the torque calc
 *      - Change x and y to be in body coordinates at the start
 *      - Physical meaning for the chosen gains?
 *      x Include angular rates into footstep reference
 *      x Online learning of pitch correction
 */

// Header File
#include "RegularizedPredictiveController.hpp"

using namespace Ipopt;

/*=======================================================================*/
/*
 * Initialize all parameters to avoid seg fault errors and accessing
 * incorrect data later on. Sets all needed parameters to be reasonable
 * values. Also initializes the LCM stream for the RPC.
 */

/**
 * General constructor for the Regularized Predictive Controller Object.
 * Initializes all of the parameters for all parts of the problem.
 */
RegularizedPredictiveController::RegularizedPredictiveController() {

  // Set up the LCM
  lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");

  // Check that the LCM object is good
  if (lcm->good()) {
    printf("LCM IN RPC INITIALIZED\n");

    // Initialize the LCM field values
    rpc_data_publish.cpu_opt_time_microseconds = 0;
    rpc_data_publish.t_sent = 0;
    for (int k = 0; k < NUM_LCM_PRED_OUT; k++) {
      rpc_data_publish.dt_pred[k] = 0;

      for (int ind = 0; ind < NUM_DECISION_VARS; ind++) {
        if (ind < NUM_STATES) {
          rpc_data_publish.x_opt[k][ind] = 0;
        } else {
          rpc_data_publish.u_opt[k][ind - NUM_STATES] = 0;
        }
      }
    }
  } else {
    printf("LCM IN RPC FAILED\n");
    exit(-1);
  }

  // Start timer
  clock_timer = std::clock();
}


/**
 * Create bounds for the decision variables and the constraints.
 * Note that the first timestep states are constrained to the robot's
 * state at the beginning of the optimization.
 */
void RegularizedPredictiveController::Bounds(double* X_ub, double* X_lb, double* constraints_ub, double* constraints_lb) {

  // Create the bounds for the prediction horizon
  for (int k = 0; k < NUM_PREDICTIONS; k++) {
    i_xd = k * NUM_STATES; // eventually get an input trajectory
    i_X = k * NUM_DECISION_VARS;
    i_c = k * NUM_CONSTRAINTS;
    i_f = k * NUM_FEET;

    // First step bounds have to be current state
    if (k == 0) {
      double zero_states[NUM_STATES] = {0};
      double zero_hip[3 * NUM_FEET] = {0};

      RPCBounds(current_state, zero_states, contact_state_pred + i_f, zero_hip,
                current_state, current_state, inputs_max_0, inputs_min_0,
                X_lb, X_ub, constraints_ub, constraints_lb);

    } else {
      RPCBounds(x_d + i_xd, x_d + i_xd, contact_state_pred + i_f, r_hip, states_max, states_min, inputs_max, inputs_min,
                X_lb + i_X, X_ub + i_X, constraints_ub + i_c, constraints_lb + i_c);
    }
  }
  /*
  cout << "NEW Bounds:" << endl;
  for (int ind = 0; ind < NUM_PREDICTIONS; ind++) {
    for (int ind2 = 0; ind2 < NUM_DECISION_VARS; ind2++) {
      cout << X_lb[ind * NUM_DECISION_VARS + ind2] << " " << X_ub[ind*NUM_DECISION_VARS + ind2] << endl;
      //cout << std::right << std::setw(13) << X[ind + ind2*NUM_DECISION_VARS];
    }
  }
  cout << "\n\n" << endl;*/
}

/**
 * Sets the seeded state trajectory, x_seed, and the seeded input reference
 * policy, u_seed, by calculating the reference policy, (r_ref, f_ref), for
 * all of the feet throughout the prediction horizon trajectory.
 *
 * TODO:
 *  - Add turning feed forward
 */
void RegularizedPredictiveController::InitializeRPC(double* X_0) {

  // Set up a temporary vector to hold the current output
  double x_0[NUM_DECISION_VARS] = {0};

  // Pull original foot locations from LCM
  double p_foot[3 * NUM_FEET] = {0};

  // Copy the current state into the temporary variable
  for (int i = 0; i < NUM_STATES; i++) {
    x_0[i] = current_state[i];
  }

  // Copy the current foot positions into the temporary foothold variable
  for (int i = 0; i < 3 * NUM_FEET; i++) {
    p_foot[i] = p_foot_0[i];
  }

  //cout << contact_state_pred[0] << " " << " " << touchdown_pred[0] << " "  << T_stance[0] << " " << x_d[6] << endl;
  // Create the initial guess and policy seeding
  for (int k = 0; k < NUM_PREDICTIONS; k++) {

    // Start indices
    i_x = k * NUM_DECISION_VARS;
    i_x1 = (k + 1) * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_xd = k * NUM_STATES; // eventually get an input trajectory
    i_xref = k * NUM_STATES;
    i_uref = k * NUM_INPUTS;
    i_f = k * NUM_FEET;
    i_H_X = k * NUM_DECISION_VARS;

    /*for (int i = 0; i < NUM_FEET; i++) {

        cout << gravity << " " << T_stance[0] <<" " << touchdown_pred[i_f+i]<<" " <<p_foot_0[i*3+0] << " " << p_foot_0[i*3+1] << " " << p_foot_0[i*3+2] << endl;;
    }*/
    // Set the current decision variables for the states
    for (int i = 0; i < NUM_STATES; i++) {
      X_0[i_x + i] = x_0[i];
    }

    // Mark the location at the beginning of touchdown
    for (int foot = 0; foot < NUM_FEET; foot++) {
      if (touchdown_pred[i_f + foot] == 1 || k == 0) {
        for (int i = 0; i < 3; i++) {
          x_touchdown[foot * 3 + i] = x_0[i]; // CHECK THIS... Prob in testing
        }
      }
    }

    RPCHeuristics(X_0 + i_x, x_d + i_xd, contact_state_pred + i_f, s_Phi1 + i_f, r_hip, stance_fraction,
                  flight_phase[k], T_stance,  K_ref, mass, gravity, z_g, H_X + i_H_X);
    RPCInitialize(X_0 + i_x, x_d + i_xd, contact_state_pred + i_f, H_X + i_H_X, p_foot,
                  touchdown_pred + i_f, x_touchdown,
                  dt_pred[k], mass, I_body_diagonal, gravity,
                  x_0, x_ref + i_xref, u_ref + i_uref, p_foot);

    // Iterative initialization function
    /*
    RPCInitialize(X_0 + i_x, x_d + i_xd, contact_state_pred + i_f, r_hip, p_foot,
                  touchdown_pred + i_f, x_touchdown,
                  dt_pred[k], stance_fraction, flight_phase[k], T_stance,
                  K_ref, mass, I_body_diagonal, gravity,
                  x_0, u_ref + i_uref, p_foot);
      */
    /*for (int i = 0; i < NUM_FEET; i++) {
        cout << p_foot[i*3+0] << " " << p_foot_0[i*3+1] << " " << p_foot_0[i*3+2];
        cout << endl;
    }*/
    /*
            for (int i = 0; i < NUM_FEET; i++) {

                cout << gravity << " " << T_stance[0] <<" " << touchdown_pred[i_f+i]<<" " <<p_foot_0[i*3+0] << " " << p_foot_0[i*3+1] << " " << p_foot_0[i*3+2] << endl;;
            }
            cout<< endl;*/

    // Set the current decision variables for the inputs
    for (int i = 0; i < NUM_INPUTS; i++) {
      X_0[i_u + i] = x_0[NUM_STATES + i];
    }
  }

  /*
  cout << "initializing" << endl;
  for (int ind = 0; ind < NUM_INPUTS; ind++){
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
          cout << std::right << std::setw(13) << u_ref[ind + ind2*NUM_INPUTS];
      }
      cout << "" << endl;
  }
  */
  /*
  cout << "\nX0" << endl;
  for (int ind = 0; ind < NUM_DECISION_VARS; ind++){
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
          cout << std::right << std::setw(13) << X_0[ind + ind2*NUM_DECISION_VARS];
      }
      cout << "" << endl;
  }
    */
  /*
  cout << "\nTD" << endl;
  for (int ind = 0; ind < NUM_FEET; ind++){
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
          cout << std::right << std::setw(13) << touchdown_pred[ind + ind2*NUM_FEET];
      }
      cout << "" << endl;
  }
  cout << "\n" << endl;*/
  /*for (int i = 0; i < NUM_PREDICTIONS; i++) {

    cout << contact_state_pred[NUM_FEET*i + 0] << " " << contact_state_pred[NUM_FEET*i + 1] << " " << contact_state_pred[NUM_FEET*i + 2] << " " << contact_state_pred[NUM_FEET*i + 3] << endl;

  }*/
  //cout << K_ref[20];
  //cout << "\n\n" << endl;
}

/**
 * Predicted cost over the given gait-generalized prediction horizon.
 * Calls the RPCCost function that is generated first symbolically in
 * MATLAB to produce an optimized MATLAB function, then a function is
 * created using MATLAB's codegen. This method is essentially a wrapper
 * that takes the problem data and uses this iterative cost function with
 * the known gait logic.
 *
 *
 *  J(X) = x_error'*Q*x_error + u_error'*R*u_error
 *
 *      x_error = x_d - x
 *      u_error = u_ref - u
 */
double RegularizedPredictiveController::PredictedCost(const double * X) {

  // Initialize the predicted cost
  double cost = 0;

  // Incremental cost over the predicted gait horizon
  for (int k = 0; k < NUM_PREDICTIONS; k++) {

    // Start indices
    i_x = k * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_xd = k * NUM_STATES; // eventually get an input trajectory
    i_xref = k * NUM_STATES;
    i_uref = k * NUM_INPUTS;
    i_f = k * NUM_FEET;
    i_Q = k * NUM_STATES;
    i_R = k * NUM_INPUTS;


    // Calculate the incremental cost
    cost = cost + RPCCost(X + i_x, X + i_u, x_ref + i_xref, u_ref + i_uref,
                          Q + i_Q, R + i_R);
    //cost = cost + RPCCost(X + i_x, X + i_u, x_d + i_xd, u_ref + i_uref,
    //                      Q + i_Q, R + i_R);
  }
  /*
  for (int ind = 0; ind < NUM_DECISION_VARS; ind++){
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
          cout << std::right << std::setw(13) << X[ind + ind2*NUM_DECISION_VARS];
      }
      cout << "" << endl;
  }*/

  //std::usleep(10000000);
  // Return the cost over the prediction horizon
  return cost;
}


/**
 * Predicted constrints over the given gait-generalized prediction horizon.
 */
void RegularizedPredictiveController::Constraints(const double * X, double * constraints) {
  //cout << "NEW ITER" << endl;
  // Run the iterative constraints
  for (int k = 0; k < NUM_PREDICTIONS; k++) {

    // Start indices
    i_x = k * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_f = k * NUM_FEET;
    i_c = k * NUM_CONSTRAINTS;
    i_lf = k * NUM_FEET;
    i_f1 = (k + 1) * NUM_FEET;
    if (k == (NUM_PREDICTIONS - 1)) {
      // Evaluate the constraints at the final timestep
      RPCConstraintsFinal(X + i_x, X + i_u,
                          contact_state_pred + i_f, lifted_foot + i_lf, z_g, mu,
                          constraints + i_c);

    } else {
      i_x1 = (k + 1) * NUM_DECISION_VARS;
      i_u1 = i_x1 + NUM_STATES;

      // Evaluate the constraints at the timestep
      RPCConstraints(X + i_x, X + i_u, X + i_x1, X + i_u1, contact_state_pred + i_f,
                     contact_state_pred + i_f1, lifted_foot + i_lf, dt_pred[k],
                     mass, I_body_diagonal, gravity, z_g, mu,
                     constraints + i_c);
    }
    /*
    for (int i = 0; i < NUM_FEET; i++) {
        cout << " ground: " << z_g[i] << " r's: " << X[i_u+2 + i*6] << " z:" << X[i_x+2] << " pz:" << X[i_x+2] + X[i_u+2 + i*6];
    }
    cout << endl;

    for (int ind = 0; ind < NUM_CONSTRAINTS; ind++) {
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++) {
        cout << std::right << std::setw(13) << constraints[ind + ind2 * NUM_CONSTRAINTS];
      }
      cout << "" << endl;
    }
    cout << "\n\n" << endl;
    */
  }
}


/**
 * Predicted cost gradient over the given gait-generalized prediction horizon.
 */
void RegularizedPredictiveController::PredictedCostGradient(const double * X, double * gradient) {

  // Incremental gradient over the predicted gait horizon
  for (int k = 0; k < NUM_PREDICTIONS; k++) {
    // Start indices
    i_x = k * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_uref = k * NUM_INPUTS;
    i_g = k * NUM_DECISION_VARS;
    i_f = k * NUM_FEET;
    i_Q = k * NUM_STATES;
    i_R = k * NUM_INPUTS;

    // Calculate the iterative gradient of the cost w.r.t. the decision variables
    RPCCostGradient(X + i_x, X + i_u, x_ref + i_xref, u_ref + i_uref,
                    Q + i_Q, R + i_R, gradient + i_g);
    /*for (int i = 0; i < NUM_INPUTS; i++) {
        cout << u_ref[i_uref + i] << endl;
    }*/

  }
  /*
  cout << "NEW GRADIENT" << endl;
  for (int i = 0; i < NUM_PREDICTIONS; i++) {
    for (int j = 0; j < NUM_DECISION_VARS; j++) {
      cout << gradient[i * NUM_DECISION_VARS + j] << endl;
    }
  }
  cout << "\n\n" << endl;
    */
}


/**
 * Predicted cost over the given gait-generalized prediction horizon.
 */
void RegularizedPredictiveController::ConstraintJacobian(const double * X, double * constraint_jacobian) {

  // Incremental cost over the predicted gait horizon
  for (int k = 0; k < NUM_PREDICTIONS; k++) {
    i_jc = k * NNZ_J_G;
    i_x = k * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_f = k * NUM_FEET;
    i_c = k * NUM_CONSTRAINTS;
    i_lf = k * NUM_FEET;
    if (k == (NUM_PREDICTIONS - 1)) {
      i_x1 = i_x;
      i_u1 = i_u;
    } else {
      i_x1 = (k + 1) * NUM_DECISION_VARS;
      i_u1 = i_x1 + NUM_STATES;
    }
    i_f1 = (k + 1) * NUM_FEET;

    if (k == (NUM_PREDICTIONS - 1)) {
      // Final constraints only dependent on contact state and friction
      RPCConstraintJacobianFinal(contact_state_pred + i_f, lifted_foot + i_lf, mu,
                                 constraint_jacobian + i_jc);
    } else {
      RPCConstraintJacobian(X + i_x, X + i_u, contact_state_pred + i_f,
                            contact_state_pred + i_f1, lifted_foot + i_lf, dt_pred[k],
                            mass, I_body_diagonal, mu,
                            constraint_jacobian + i_jc);

    }
  }
}


/**
 * Predicted lagrangian hessian over the given gait-generalized prediction horizon.
 */
void RegularizedPredictiveController::LagrangianHessian(const double * X, double * hessian, Number obj_factor, const Number * lambda) {

  // Incremental cost over the predicted gait horizon
  for (int k = 0; k < NUM_PREDICTIONS; k++) {

    // Start indices
    i_x = k * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_c = k * NUM_CONSTRAINTS;
    i_h = k * NNZ_H;
    i_f = k * NUM_FEET;
    i_Q = k * NUM_STATES;
    i_R = k * NUM_INPUTS;

    if (k == (NUM_PREDICTIONS - 1)) {
      // Calculate the iterative Hessian of the Lagrangian at the lfinal time
      RPCLagrangianHessianFinal(Q + i_Q, R + i_R, obj_factor,
                                hessian + i_h);
    } else {
      // Calculate the iterative Hessian of the Lagrangian over the prediction horizon
      RPCLagrangianHessian(X + i_x, X + i_u, contact_state_pred + i_f,
                           Q + i_Q, R + i_R, dt_pred[k],
                           obj_factor, lambda + i_c, I_body_diagonal,
                           hessian + i_h);
    }
  }
}



/*============================Gait Functions=============================*/
/*
 * Create a vector of reasonable timesteps during the prediction horizon.
 * Also stores the contact state for each leg during the prediciton steps.
 */
void RegularizedPredictiveController::GetPredictedTimeVector() {

  // Timing parameters
  double phi_switch[NUM_FEET];  // phase where contact switches
  double phi_pred[NUM_FEET];               // tracks the phases of each foot
  double dt_nominal = 0.1;                 // nominal timestep to take
  double dt_switch;                        // timestep till next contact switch
  double dt_max;                           // maximum timestep allowed due to contact switching
  double num_feet_swing = 0;               // tracks the number of feet in swing
  double t_flight = 0;
  double t_total = 0;
  double min_dt_pred = 0.0005;

  // Initialize the foot phases to the current actual phase
  for (int foot = 0; foot < NUM_FEET; foot++) {
    phi_switch[foot] = (T_p[foot] - T_swing[foot]) / T_p[foot];
    phi_pred[foot] = phi_0[foot];
    T_stance[foot] = T_p[foot] - T_swing[foot];
  }

  int k = 0;

  // Calculate the dt for each prediction step
  while (k < NUM_PREDICTIONS) {//for (int k = 0; k < NUM_PREDICTIONS; k++) {
    i_f = k * NUM_FEET;

    // Reset the maximum allowable timestep
    dt_max = 1;

    // Reset the number of feet in swing
    num_feet_swing = 0;

    // Find if a contact change occurs in each foot
    for (int foot = 0; foot < NUM_FEET; foot++) {

      if (enabled[foot] == 1) {
        // Find which state the foot is in
        if (phi_pred[foot] < phi_switch[foot]) {
          // In contact state
          contact_state_pred[NUM_FEET * k + foot] = 1;
          if (k > 0 && contact_state_pred[NUM_FEET * (k - 1) + foot] == 0) {
            touchdown_pred[NUM_FEET * k + foot] = 1;
            if (next_step[foot] == 0) {
              next_step[foot] = k;
            }
          } else {
            touchdown_pred[NUM_FEET * k + foot] = 0;
          }

          // Set the boolean for lifted foot logic
          if (k == 0) {
            lifted_foot[NUM_FEET * k + foot] = 0;  /////////////
          } else if (lifted_foot[NUM_FEET * (k - 1) + foot] == 1) {
            lifted_foot[NUM_FEET * k + foot] = 1;
          } else {
            lifted_foot[NUM_FEET * k + foot] = 0;            ///////////////
          }


          dt_switch = (phi_switch[foot] - phi_pred[foot]) * T_p[foot];
        } else {
          // In swing state
          contact_state_pred[NUM_FEET * k + foot] = 0;
          touchdown_pred[NUM_FEET * k + foot] = 0;
          dt_switch = (1 - phi_pred[foot]) * T_p[foot];
          lifted_foot[NUM_FEET * k + foot] = 1;
          num_feet_swing++;
        }
      } else {
        // In swing state
        contact_state_pred[NUM_FEET * k + foot] = 0;
        touchdown_pred[NUM_FEET * k + foot] = 0;
        lifted_foot[NUM_FEET * k + foot] = 1;
        dt_switch = T_p[foot];
        num_feet_swing++;
      }
      // Replace dt_max with smallest dt to a contact
      if (dt_switch < dt_max) {
        dt_max = dt_switch;
      }
    }

    // Create reasonably spaced timesteps based on contact states
    if (NUM_FEET == num_feet_swing) {
      // All feet are in swing so timestep is until next contact
      dt_pred[k] = dt_max;
      if (t_total < T_p[0]) {
        t_flight += dt_pred[k];
      }
      flight_phase[k] = 1;
    } else if (dt_max <= dt_nominal) {
      // Must end timestep at smallest timestep to contact switch
      dt_pred[k] = dt_max;
      flight_phase[k] = 0;
    } else {
      // Contact switch happens far enough away, use nominal timestep
      dt_pred[k] = dt_max / round(dt_max / dt_nominal);
      flight_phase[k] = 0;
    }
    t_total += dt_pred[k];

    // period_percent = t_total/T_p[foot];


    // Check to make sure the timestep is large enough
    if (dt_pred[k] >= min_dt_pred) {
      // If the timestep is large enough, move on to the next division
      for (int foot = 0; foot < NUM_FEET; foot++) {
        s_Phi0[i_f + foot] = phi_pred[foot];
        phi_pred[foot] = fmod((phi_pred[foot] + (dt_pred[k]) / T_p[foot]), 1);
        s_Phi1[i_f + foot] = phi_pred[foot];
      }
      k++;
    } else {
      // If the timestep is not large enough, increase timestep and try again
      for (int foot = 0; foot < NUM_FEET; foot++) {
        phi_pred[foot] = fmod((phi_pred[foot] + 2 * (dt_pred[k]) / T_p[foot]), 1);
      }
    }
    //cout << flight_phase[k] << " ";
  }

  if (T_p[0] != t_flight) {
    stance_fraction = T_p[0] / (T_p[0] - t_flight);
    //cout << stance_fraction << endl;
    //stance_fraction = T_p[0]/(T_p[0] - T_swing[0]);
  } else {
    stance_fraction = 0;
    cout << "[RPC] ERROR Calculating Stance Fraction, T_p = t_flight" << endl;
  }

  /*for (int k = 0; k < NUM_PREDICTIONS; k++) {
    for (int foot = 0; foot < NUM_FEET; foot++) {
      cout << lifted_foot[NUM_FEET * k + foot] << " ";
    }
    cout << endl;
  }
  cout << endl;
  cout << endl;*/
}



/*============================IPOPT Functions============================*/
/*
 * Methods that map the internal RPC functions to IPOPT functions.
 */

bool RegularizedPredictiveController::get_nlp_info(Ipopt::Index & n, Ipopt::Index & m, Ipopt::Index & nnz_jac_g,
    Ipopt::Index & nnz_h_lag, IndexStyleEnum & index_style) {

  //cout << "GET NLP INFO" << endl;
  SOLVED = 0;

  // Number of total decision variables for the problem
  n = NUM_DECISION_VARS * NUM_PREDICTIONS;

  // Number of total constraints for the problem
  m = NUM_CONSTRAINTS * NUM_PREDICTIONS;

  // Number of nonzeros in the constraint Jacobian
  nnz_jac_g = NNZ_J_G * (NUM_PREDICTIONS - 1) + NNZ_J_G_F; // Last step only dependent on current so dynamics goes away

  // Number of nonzeros in the Hessian matrix (number calculated from MATLAB)
  nnz_h_lag = NNZ_H * (NUM_PREDICTIONS - 1) + NNZ_H_F; // Currently only for cost hessian, need constraints
  // Will definitely change possibly... It didnt change! Maybe...

  // Index style is 0-based
  index_style = TNLP::C_STYLE;

  // Set the new variables from the user input
  SetParameters();

  //cout << "[RPC] Parameters set" << endl;

  return true;
}


bool RegularizedPredictiveController::get_bounds_info(Ipopt::Index n, Number * x_l, Number * x_u,
    Ipopt::Index m, Number * g_l, Number * g_u) {
  //cout << "Get bounds info" << endl;
  // Calculate the initial upper and lower bounds for the decision variables and constraints
  Bounds(x_u, x_l, g_u, g_l);
  return true;
}

bool RegularizedPredictiveController::get_starting_point(Ipopt::Index n, bool init_x, Number * x,
    bool init_z, Number * z_L, Number * z_U,
    Ipopt::Index m, bool init_lambda,
    Number * lambda) {

  /*cout << "initialize " << endl;
  for (int ind = 0; ind < NUM_DECISION_VARS; ind++){
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
          cout << std::right << std::setw(13) << x[ind + ind2*NUM_DECISION_VARS];
      }
      cout << "" << endl;
  }
  cout << "\n\n" << endl;*/
  // Initialize the decision variable guess
  InitializeRPC(x);

  /*
  for (int ind = 0; ind < NUM_DECISION_VARS; ind++){
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
          cout << std::right << std::setw(13) << x[ind + ind2*NUM_DECISION_VARS];
      }
      cout << "" << endl;
  }
  cout << "\n\n" << endl;
  */
  return true;
}

bool RegularizedPredictiveController::eval_f(Ipopt::Index n, const Number * x, bool new_x, Number & obj_value) {
  //cout << "[RPC] Cost Calculation Start" << endl;
  // Solve the Predicted cost function
  obj_value = PredictedCost(x);
  //cout << "[RPC] Cost Calculation End" << endl;
  return true;
}

bool RegularizedPredictiveController::eval_grad_f(Ipopt::Index n, const Number * x, bool new_x, Number * grad_f) {
  //cout << "[RPC] Cost Gradient Calculation Start" << endl;
  // Calculate the cost gradient
  PredictedCostGradient(x, grad_f);
  //cout << "[RPC] Cost Gradient Calculation End" << endl;

  return true;
}

bool RegularizedPredictiveController::eval_g(Ipopt::Index n, const Number * x, bool new_x, Ipopt::Index m, Number * g) {
  //cout << "[RPC] Constraint Calculation Start" << endl;

  // Calculate the set of constraints
  Constraints(x, g);
  //cout << "[RPC] Constraint Calculation End" << endl;


  return true;
}

bool RegularizedPredictiveController::eval_jac_g(Ipopt::Index n, const Number * x, bool new_x,
    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index * iRow, Ipopt::Index * jCol,
    Number * values) {
  if (values == NULL) {
    //cout << "[RPC] Constraint Jacobian Setup Start" << endl;

    for (int i = 0; i < NUM_PREDICTIONS; i++) {
      i_jc = i * NNZ_J_G;
      if (i < (NUM_PREDICTIONS - 1)) {
        RPCConstraintJacobianSP(i, NUM_DECISION_VARS, NUM_CONSTRAINTS, iRow + i_jc, jCol + i_jc);
      } else {
        RPCConstraintJacobianFinalSP(i, NUM_DECISION_VARS, NUM_CONSTRAINTS, iRow + i_jc, jCol + i_jc);
      }
    }
    //cout << "[RPC] Constraint Jacobian Setup End" << endl;

  } else {
    //cout << "[RPC] Constraint Jacobian Calculation Start" << endl;

    // Calculate the constraint jacobian
    ConstraintJacobian(x, values);
    //cout << "[RPC] Constraint Jacobian Calculation End" << endl;

  }
  return true;
}

bool RegularizedPredictiveController::eval_h(Ipopt::Index n, const Number * x, bool new_x,
    Number obj_factor, Ipopt::Index m, const Number * lambda,
    bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index * iRow,
    Ipopt::Index * jCol, Number * values) {
  if (values == NULL) {
    //cout << "[RPC] Hessian Setup Start" << endl;

    for (int i = 0; i < NUM_PREDICTIONS; i++) {
      i_h = i * NNZ_H;

      if (i < (NUM_PREDICTIONS - 1)) {
        RPCLagrangianHessianSP(i, NUM_DECISION_VARS, iRow + i_h, jCol + i_h);
      } else {
        RPCLagrangianHessianFinalSP(i, NUM_DECISION_VARS, iRow + i_h, jCol + i_h);
      }
    }
    //cout << "[RPC] Hessian Setup End" << endl;

  } else {
    //cout << "[RPC] Hessian Calculation Start" << endl;

    // Calculate the Hessian
    LagrangianHessian(x, values, obj_factor, lambda);
    //cout << "[RPC] Hessian Calculation End" << endl;

  }
  return true;
}

void RegularizedPredictiveController::finalize_solution(SolverReturn status,
    Ipopt::Index n, const Number * x, const Number * z_L, const Number * z_U,
    Ipopt::Index m, const Number * g, const Number * lambda,
    Number obj_value, const IpoptData * ip_data,
    IpoptCalculatedQuantities * ip_cq) {


  int NUM_TOTAL = 0;
  if (NUM_PREDICTIONS < NUM_PREDICTIONS_OUT) {
    NUM_TOTAL = NUM_PREDICTIONS;
  } else {
    NUM_TOTAL = NUM_PREDICTIONS_OUT;
  }


  //pthread_mutex_lock(&solve_mutex);
  for (int k = 0; k < NUM_TOTAL; k++) {
    dt_pred_final[k] = dt_pred[k];

    // Add the time vector to the publishing data
    rpc_data_publish.dt_pred[k] = dt_pred_final[k];
    for (int ind = 0; ind < NUM_DECISION_VARS; ind++) {
      X_final[k * NUM_DECISION_VARS + ind] = x[k * NUM_DECISION_VARS + ind];

      if (ind < NUM_STATES) {
        rpc_data_publish.x_opt[k][ind] = X_final[k * NUM_DECISION_VARS + ind];
      } else {
        rpc_data_publish.u_opt[k][ind - NUM_STATES] = X_final[k * NUM_DECISION_VARS + ind];
      }

    }
  }

  for (int foot = 0; foot < NUM_FEET; foot++) {
    for (int i = 0; i < 3; i++) {
      if (next_step[foot] == 0) {
        p_foot_f[foot * 3 + i] = 0;
      } else {
        p_foot_f[foot * 3 + i] = x[next_step[foot] * NUM_DECISION_VARS + NUM_STATES + foot * 6 + i];
      }
      // Add the optimal next foot locations to the publishing data
      rpc_data_publish.p_opt[i][foot] = p_foot_f[foot * 3 + i];
    }
    next_step[foot] = 0;
  }

  SOLVED = 1;
  NEW_SOLUTION = 1;
  //cout << "SOLVED!" << endl;
  if (false) {
    /*
    cout << "Final Solution, X_final:" << endl;
    for (int ind = 0; ind < NUM_DECISION_VARS; ind++) {
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++) {
        cout << std::right << std::setw(13) << X_final[ind + ind2 * NUM_DECISION_VARS];
      }
      cout << "" << endl;
    }
    cout << "\nCost: " << obj_value << "\n\n" << endl;
    */
    /*
    cout << "r_hip: " <<  r_hip[0] << " " << r_hip[1] << "\n" << endl;
    */
    /*
    cout << "Input Reference, u_ref:" << endl;
    for (int ind = 0; ind < NUM_INPUTS; ind++) {
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++) {
        cout << std::right << std::setw(13) << u_ref[ind + ind2 * NUM_INPUTS];
      }
      cout << "" << endl;
    }
    cout << "\n\n" << endl;
    */

    /*
    cout << "State Desired, x_d:" << endl;
    for (int ind = 0; ind < NUM_STATES; ind++) {
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++) {
        cout << std::right << std::setw(13) << x_d[ind + ind2 * NUM_STATES];
      }
      cout << "" << endl;
    }
    cout << "\n\n" << endl;
    //pthread_mutex_unlock(&solve_mutex);

    cout << "Time vector, dt:" << endl;
    for (int i = 0; i < NUM_PREDICTIONS; i++) {
      cout << dt_pred_final[i] << "     ";
    }
    cout << "\n\n" << endl;
    */

    /*
    cout << "Constraints, g(x):" << endl;
    for (int ind = 0; ind < NUM_CONSTRAINTS; ind++) {
      for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++) {
        cout << std::right << std::setw(13) << g[ind + ind2 * NUM_CONSTRAINTS];
      }
      cout << "" << endl;
    }
    cout << "\n\n" << endl;
    */
    /*
    cout << "Lambdas:" << endl;
    for (int ind = 0; ind < NUM_CONSTRAINTS; ind++){
        for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
            cout << std::right << std::setw(13) << lambda[ind + ind2*NUM_CONSTRAINTS];
        }
        cout << "" << endl;
    }
    cout << "\n\n" << endl;
    */
    // Maybe add timing statistics
    // Might also have to cancel out the annoying forces that appear due to interior point algorithm
  }

  //rpc_data_publish.time_start = time_start;
  rpc_data_publish.t_sent = std::clock();
  rpc_data_publish.cpu_opt_time_microseconds = rpc_data_publish.t_sent - clock_timer;

  // LCM solution back out to the main code
  PublishDataLCM();


  clock_timer = std::clock();
}




void RegularizedPredictiveController::SetCurrentState(double * current_state_in) {
  //pthread_mutex_lock(&solve_mutex);
  for (int i = 0; i < NUM_STATES; i++) {
    current_state_new[i] = current_state_in[i];
  }
  //pthread_mutex_unlock(&solve_mutex);
}

void RegularizedPredictiveController::SetFootLocations(double * p_foot_0_in) {
  //pthread_mutex_lock(&solve_mutex);
  for (int foot = 0; foot < NUM_FEET; foot++) {
    for (int i = 0; i < 3; i++) {
      if (i == 0 || i == 1) {
        p_foot_0_new[foot * 3 + i] = p_foot_0_in[foot * 3 + i]; // - current_state[i];
      } else {
        p_foot_0_new[foot * 3 + i] = p_foot_0_in[foot * 3 + i];
      }
    }
  }
  //pthread_mutex_unlock(&solve_mutex);
}

void RegularizedPredictiveController::SetHipLocations(double * r_hip_in) {
  //pthread_mutex_lock(&solve_mutex);
  for (int foot = 0; foot < NUM_FEET; foot++) {
    for (int axis = 0; axis < 3; axis++) {
      r_hip_new[foot * 3 + axis] = r_hip_in[foot * 3 + axis];
    }
  }
  //pthread_mutex_unlock(&solve_mutex);
}

void RegularizedPredictiveController::SetDesiredTrajectory(double * x_d_in) {
  //pthread_mutex_lock(&solve_mutex);
  for (int i = 0; i < NUM_PREDICTIONS * NUM_STATES; i++) {
    x_d_new[i] = x_d_in[i];
  }
  //pthread_mutex_unlock(&solve_mutex);
}

/**
 * Creates a new state error weighting matrix.
 *
 *  @param newQ
 *      the new matrix values for Q in an array.
 */
void RegularizedPredictiveController::SetStateWeights(double* Q_in)
{
  //pthread_mutex_lock(&solve_mutex);
  for (int i = 0; i < NUM_STATES; i++) {
    Q_new[i] = Q_in[i];
  }
  //pthread_mutex_unlock(&solve_mutex);
}

/**
 * Creates a new input regularization weighting matrix.
 *
 *  @param newR
 *      the new matrix values for R in an array.
 */
void RegularizedPredictiveController::SetInputRegularization(double* R_in)
{
  //pthread_mutex_lock(&solve_mutex);
  int k = 0;
  for (int foot = 0; foot < NUM_FEET; foot++) {
    for (int i = 0; i < 6; i++) {
      R_new[k] = R_in[i];
      k++;
    }
  }
  //pthread_mutex_unlock(&solve_mutex);
}

/**
 * Creates a new state error weighting matrix.
 *
 *  @param newQ
 *      the new matrix values for Q in an array.
 */
void RegularizedPredictiveController::SetHeuristicGains(double* K_ref_in)
{
  //pthread_mutex_lock(&solve_mutex);
  for (int i = 0; i < 30; i++) {
    K_ref_new[i] = K_ref_in[i];
  }
  //pthread_mutex_unlock(&solve_mutex);
}

void RegularizedPredictiveController::SetRobotParameters(double mass_in, double* Inertia_in) {
  //pthread_mutex_lock(&solve_mutex);
  mass_new = mass_in;
  for (int i = 0; i < 3; i++) {
    Inertia_new[i] = Inertia_in[i];
  }
  //pthread_mutex_unlock(&solve_mutex);
  //cout << "MASS: " << mass << "   |   INERTIA: " << I_body_diagonal[0]<< I_body_diagonal[1]<< I_body_diagonal[2]<< endl;
}

void RegularizedPredictiveController::SetGaitParameters(double* enabled_in, double* phi_0_in, double* T_p_in, double* T_swing_in) {

  //pthread_mutex_lock(&solve_mutex);
  for (int foot = 0; foot < NUM_FEET; foot++) {
    enabled_new[foot] = enabled_in[foot];
    phi_0_new[foot] = phi_0_in[foot];
    T_p_new[foot] = T_p_in[foot];
    T_swing_new[foot] = T_swing_in[foot];
  }
  //pthread_mutex_unlock(&solve_mutex);
  //GetPredictedTimeVector();
}

void RegularizedPredictiveController::SetEnvironmentParameters(double* gravity_in, double* mu_in, double* z_g_in) {
  //pthread_mutex_lock(&solve_mutex);

  for (int axes = 0; axes < 3; axes++) {
    gravity_new[axes] = gravity_in[axes];
  }

  for (int foot = 0; foot < NUM_FEET; foot++) {
    mu_new[foot] = mu_in[foot];
    z_g_new[foot] = z_g_in[foot];
  }
  //pthread_mutex_unlock(&solve_mutex);
}

void RegularizedPredictiveController::SetTimeStart(double time_start_in) {
  //pthread_mutex_lock(&solve_mutex);
  time_start_new = time_start_in;
  //pthread_mutex_unlock(&solve_mutex);
}

void RegularizedPredictiveController::SetParameters() {

  //pthread_mutex_lock(&solve_mutex);

  // Set Current State
  for (int i = 0; i < NUM_STATES; i++) {
    current_state[i] = current_state_new[i];
  }

  // Desired Trajectory
  for (int i = 0; i < NUM_PREDICTIONS * NUM_STATES; i++) {
    x_d[i] = x_d_new[i];
  }

  // Heuristic Gains
  for (int i = 0; i < 30; i++) {
    K_ref[i] = K_ref_new[i];
  }

  // Robot Parameters
  mass = mass_new;
  for (int i = 0; i < 3; i++) {
    I_body_diagonal[i] = Inertia_new[i];
  }

  // Gait Parameters
  for (int foot = 0; foot < NUM_FEET; foot++) {
    phi_0[foot] = phi_0_new[foot];
    T_p[foot] = T_p_new[foot];
    T_swing[foot] = T_swing_new[foot];
    enabled[foot] = enabled_new[foot];
  }

  // Adaptive algorithm for even timestep divisions
  GetPredictedTimeVector();

  // Adust the desired position and yaw for velocity
  for (int k = 1; k < NUM_PREDICTIONS; k++) {
    x_d[k * NUM_STATES + 5] = x_d[(k - 1) * NUM_STATES + 5]  +
                              dt_pred[k - 1] * x_d[(k - 1) * NUM_STATES + 11];
    x_d[k * NUM_STATES + 0] = x_d[(k - 1) * NUM_STATES + 0]  +
                              dt_pred[k - 1] * ( x_d[(k - 1) * NUM_STATES + 6] * std::cos( x_d[(k - 1) * NUM_STATES + 5]) -
                                  x_d[(k - 1) * NUM_STATES + 7] * std::sin( x_d[(k - 1) * NUM_STATES + 5]));
    x_d[k * NUM_STATES + 1] = x_d[(k - 1) * NUM_STATES + 1]  +
                              dt_pred[k - 1] * ( x_d[(k - 1) * NUM_STATES + 7] * std::cos( x_d[(k - 1) * NUM_STATES + 5]) +
                                  x_d[(k - 1) * NUM_STATES + 6] * std::sin( x_d[(k - 1) * NUM_STATES + 5]));
  }

  // Set foot locations
  for (int foot = 0; foot < NUM_FEET; foot++) {
    for (int i = 0; i < 3; i++) {
      if (i == 0 || i == 1) {
        p_foot_0[foot * 3 + i] = p_foot_0_new[foot * 3 + i]; // Already subtracted CoM position in MATLAB
        inputs_min_0[foot * 6 + i] = contact_state_pred[foot] * (p_foot_0[foot * 3 + i]);
        inputs_max_0[foot * 6 + i] = contact_state_pred[foot] * (p_foot_0[foot * 3 + i]);
      } else {
        p_foot_0[foot * 3 + i] = p_foot_0_new[foot * 3 + i]; // - current_state[i];
        inputs_min_0[foot * 6 + i] = contact_state_pred[foot] * (p_foot_0[foot * 3 + i] - current_state[i]);
        inputs_max_0[foot * 6 + i] = contact_state_pred[foot] * (p_foot_0[foot * 3 + i] - current_state[i]);
      }
      r_hip[foot * 3 + i] = r_hip_new[foot * 3 + i];
      //cout << p_foot_0[foot*3 + i] << endl;
    }
    //cout << "" << endl;
  }
  //cout << "\n" << endl;

  // Environment Parameters
  for (int axes = 0; axes < 3; axes++) {
    gravity[axes] = gravity_new[axes];
  }

  for (int foot = 0; foot < NUM_FEET; foot++) {

    mu[foot] = mu_new[foot];
    z_g[foot] = z_g_new[foot];
  }

  // Record initial time for the optimization
  rpc_data_publish.time_start = time_start_new;

  // Modify the regularization and weightings
  // Move to cost function.... or not...
  // make Q and R big arrays for N timesteps???
  SetModifiedRegularization();

  //pthread_mutex_unlock(&solve_mutex);
}

void RegularizedPredictiveController::SetModifiedRegularization() {

  for (int k = 0; k < NUM_PREDICTIONS; k++) {
    i_Q = k * NUM_STATES;
    i_R = k * NUM_INPUTS;

    double K_dt = 0;
    if (k == 0) {
      K_dt = 1;// - k/NUM_PREDICTIONS;
    } else {
      K_dt = 1;// - k/NUM_PREDICTIONS;
    }

    // Modify the state weightings
    for (int i = 0; i < NUM_STATES; i++) {
      if (i == 3) {
        // Roll
        Q[i_Q + i] = K_dt * Q_new[i]; // * pow(I_body_diagonal[0], 2) / pow(dt_pred[k], 2);
      } else if (i == 4) {
        // Pitch
        Q[i_Q + i] = K_dt * Q_new[i]; // * pow(I_body_diagonal[1], 2) / pow(dt_pred[k], 2);
      } else if (i == 5) {
        // Yaw
        Q[i_Q + i] = K_dt * Q_new[i]; // * pow(I_body_diagonal[2], 2) / pow(dt_pred[k], 2);
      } else if (i == 9) {
        // dRoll
        Q[i_Q + i] = K_dt * Q_new[i]; // * pow(body_width, 4) / dt_pred[k];
      } else if (i == 10) {
        // dPitch
        Q[i_Q + i] = K_dt * Q_new[i]; // * pow(body_length, 4) / dt_pred[k];
      } else if (i == 11) {
        // dYaw
        Q[i_Q + i] = K_dt * Q_new[i]; // * pow(body_length, 4) / dt_pred[k];
      } else {
        Q[i_Q + i] = K_dt * Q_new[i];
      }
    }

    // Modify the input regularization
    int k_R = 0;
    for (int foot = 0; foot < NUM_FEET; foot++) {
      for (int i = 0; i < 6; i++) {
        if (i == 0) {
          // r_ref_x
          R[i_R + k_R] = K_dt * R_new[i]; // * 2500 * dt_pred[k] * pow(NUM_PREDICTIONS,2.5);
        } else if (i == 1) {
          // r_ref_y
          R[i_R + k_R] = K_dt * R_new[i]; // * 2500 * dt_pred[k] * pow(NUM_PREDICTIONS,2.5);
        } else if (i == 2) {
          // r_ref_z
          R[i_R + k_R] = K_dt * R_new[i]; // * 2500 * dt_pred[k] * pow(NUM_PREDICTIONS,2.5);
        } else {
          R[i_R + k_R] = K_dt * R_new[i];
        }
        k_R++;
      }
    }
  }
}


// Get the final solution
double RegularizedPredictiveController::GetSolution(int index) {
  //cout << X_final[index-1] << endl;
  return X_final[index];
}


// Get the final solution
double RegularizedPredictiveController::GetStepLocation(int index) {
  //cout << X_final[index-1] << endl;
  return p_foot_f[index];
}


double RegularizedPredictiveController::GetSolutionTime(int index) {
  return dt_pred_final[index];
}

double RegularizedPredictiveController::GetSolveTime() {
  return rpc_data_publish.cpu_opt_time_microseconds;
}


int RegularizedPredictiveController::CheckNewSolution() {
  if (NEW_SOLUTION == 1) {
    //cout << "NEW SOLUTION" << endl;
    //NEW_SOLUTION = 0;
    return 1;
  } else {
    //cout << "NO NEW SOLUTION" << endl;
    return 1;//NEW_SOLUTION;
  }

}

int RegularizedPredictiveController::GetSolved() {
  //pthread_mutex_lock(&solve_mutex);
  return SOLVED;
  //pthread_mutex_unlock(&solve_mutex);
}

/*void RegularizedPredictiveController::SetMutex(pthread_mutex_t solve_mutex) {
  solve_mutex = solve_mutex;
}*/


void RegularizedPredictiveController::PublishDataLCM() {
  lcm->publish("CONTROLLER_rpc_data", &rpc_data_publish);
}
