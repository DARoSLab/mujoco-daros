
#include "RPCInterface.h"
#include <state_machine/ControlFSM.h>

RPCInterface::RPCInterface(ControlFSMData<float>* controlFSMData) {
  data = controlFSMData;
  // Initialize inputs to -1 to signify result has not come in
  for (int k = 0; k < 5; k++) {
    for (int i = 0; i < 24; i++) {
      u_opt[k * 24 + i] = -1;
    }
  }

  // Set up the LCM
  lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");

  if (lcm->good()) {
    printf("LCM IN RPC INITIALIZED\n");
  } else {
    printf("LCM IN RPC FAILED\n");
    exit(-1);
  }
}


/**
 *
 */
void RPCInterface::setRPCInputs() {
  // Find the expected solve prediciton compensation time
  dt_solve_pred = data->userParameters->RPC_use_pred_comp
                  * std::min(((1 - alpha_solve_pred) * dt_solve_pred + alpha_solve_pred * dt_solve_k), 0.025);

  // Set the floating base states from the state estimator: x = [p, rpy, dp, drpy]
  Vec3<float> position = data->_stateEstimator->getResult().position;
  Vec3<float> rpy = data->_stateEstimator->getResult().rpy;
  //Mat3<float> Phi = OmegaTodRPY(rpy);
  Vec3<float> vWorld = data->_stateEstimator->getResult().vWorld;
  Vec3<float> drpy = data->_stateEstimator->getResult().omegaBody; //Phi * (data->_stateEstimator->getResult().rBody*data->_stateEstimator->getResult().omegaBody);

  for (int i = 0; i < 3; i++) {
    current_state[0 + i] = (double)(position(i));
    current_state[3 + i] = (double)(rpy(i));
    current_state[6 + i] = (double)(vWorld(i));
    current_state[9 + i] = (double)(drpy(i));
  }
  
  // Set the robot Kinematics
  Vec3<float> p_leg_controller;
  success_test = 1;
  for (size_t foot = 0; foot < pat_biped::num_legs; foot++) {
    p_leg_controller = data->_legController->datas[foot]->p;
    if (p_leg_controller(0) == 0 || p_leg_controller(1) == 0 || p_leg_controller(2) == 0) {
      success_test = 0;
    }

    for (int axis = 0; axis < 3; axis++) {
      r_hip[foot * 3 + axis] = (double)data->_pat->getHipLocation(foot)(axis);
      p_foot0[foot * 3 + axis] = (double)data->_stateEstimator->getFootPosCoMOrigin(foot)(axis);
    }
    // Add an offset to the y hip direction
    r_hip[foot * 3 + 1] = r_hip[foot * 3 + 1] + pow(-1, foot + 1) * data->userParameters->Swing_step_offset(1);
  }

  // Set Cost weights
  for (int axis = 0; axis < 3; axis++) {
    Q[0 + axis] = data->userParameters->RPC_Q_p[axis];       // position error cost
    Q[3 + axis] = data->userParameters->RPC_Q_theta[axis];   // orientation error cost
    Q[6 + axis] = data->userParameters->RPC_Q_dp[axis];      // translational velocity error cost
    Q[9 + axis] = data->userParameters->RPC_Q_dtheta[axis];  // angular velocity error cost

    R[0 + axis] = data->userParameters->RPC_R_r[axis];   // FR footstep error cost
    R[6 + axis] = data->userParameters->RPC_R_r[axis];   // FL footstep error cost
    R[12 + axis] = data->userParameters->RPC_R_r[axis];  // BR footstep error cost
    R[18 + axis] = data->userParameters->RPC_R_r[axis];  // BL footstep error cost

    R[3 + axis] = data->userParameters->RPC_R_f[axis];   // FR foot force error cost
    R[9 + axis] = data->userParameters->RPC_R_f[axis];   // FL foot force error cost
    R[15 + axis] = data->userParameters->RPC_R_f[axis];  // BR foot force error cost
    R[21 + axis] = data->userParameters->RPC_R_f[axis];  // BL foot force error cost
  }

  // Set the heuristic regularization weights
  for (int i = 0; i < 30; i++) {
    K_HX[i] = K_HX[i];
  }

  // Heuristics for linear lateral velocity induced roll error
  K_HX[0] = data->userParameters->RPC_H_theta0[0];
  K_HX[1] = data->userParameters->RPC_H_theta0[1];

  // Heuristics for linear forward velocity induced pitch error
  K_HX[2] = data->userParameters->RPC_H_phi0[0];
  K_HX[3] = data->userParameters->RPC_H_phi0[1];

  // Heuristics for natural sinusoidal pitch movement
  K_HX[4] = data->userParameters->RPC_H_phi0[2];

  // Heuristics for footstep placement
  K_HX[10] = data->userParameters->RPC_H_r_trans[0];  // hip locations
  K_HX[11] = data->userParameters->RPC_H_r_trans[1];  // translational velocity
  K_HX[12] = data->userParameters->RPC_H_r_rot[0];    // rotational velocity
  K_HX[13] = data->userParameters->RPC_H_r_trans[2];  // capture point
  K_HX[14] = data->userParameters->RPC_H_r_rot[1];    // high speed turning

  //pretty_print(data->_desiredStateCommand->data.stateDes, std::cout, "[INTERFACE] DES command 1");

  // Set the maximum desired command limits
  data->_desiredStateCommand->setCommandLimits(-data->userParameters->des_dp_max[0], data->userParameters->des_dp_max[0],
      -data->userParameters->des_dp_max[1], data->userParameters->des_dp_max[1],
      -data->userParameters->des_dtheta_max[2], data->userParameters->des_dtheta_max[2]);


  // Set the desired trajectory (Will eventually come from heuristic models)
  for (int k = 0; k < 5; k++) {

    if (data->userParameters->use_rc) {
      const rc_control_settings* rc_cmd = data->_desiredStateCommand->rcCommand;

      x_desired[k * 12 + 6] = rc_cmd->v_des[0]; // x vel
      x_desired[k * 12 + 7] = rc_cmd->v_des[1] * 0.6; // y vel
      x_desired[k * 12 + 8] = 0.; // z vel
      x_desired[k * 12 + 9] = 0.; // Roll rate
      x_desired[k * 12 + 10] = 0.; // Pitch rate
      x_desired[k * 12 + 11] = rc_cmd->omega_des[2]; // Yaw rate
      x_desired[k * 12 + 2] = 0.29 + rc_cmd->height_variation * 0.08;  // Height
      x_desired[k * 12 + 3] = 0.; // Roll

      double pitch_cmd = rc_cmd->omega_des[1];
      if (pitch_cmd > 0.4) {
        pitch_cmd = 0.4;
      } else if (pitch_cmd < -0.4) {
        pitch_cmd = -0.4;
      }
      x_desired[k * 12 + 4] = pitch_cmd;

    } else {
      // Desired states
      x_desired[k * 12 + 2] = data->_desiredStateCommand->data.stateDes[2];
      x_desired[k * 12 + 3] = data->_desiredStateCommand->data.stateDes[3];
      x_desired[k * 12 + 4] = data->_desiredStateCommand->data.stateDes[4];
      x_desired[k * 12 + 6] = data->_desiredStateCommand->data.stateDes[6];
      x_desired[k * 12 + 7] = data->_desiredStateCommand->data.stateDes[7];
      x_desired[k * 12 + 8] = data->_desiredStateCommand->data.stateDes[8];
      x_desired[k * 12 + 9] = data->_desiredStateCommand->data.stateDes[9];
      x_desired[k * 12 + 10] = data->_desiredStateCommand->data.stateDes[10];
      x_desired[k * 12 + 11] = data->_desiredStateCommand->data.stateDes[11];
    }

    // Save the desired states at low speed
    if (data->userParameters->control_mode == FSM_StateName::LOCOMOTION_RPC) {

      Vec3<float> dp_des_rot((float)x_desired[k * 12 + 6], (float)x_desired[k * 12 + 7], (float)x_desired[k * 12 + 8]);
      dp_des_rot = data->_stateEstimator->getResult().rBody * dp_des_rot; // might be rBody.transpose()... but doesn't matter for this

      px_des = BoundDesiredState(k, 0, position(0), px_des, (double)dp_des_rot(0), px_error_max);
      py_des = BoundDesiredState(k, 1, position(1), py_des, (double)dp_des_rot(1),  py_error_max);
      yaw_des = BoundDesiredState(k, 5, rpy(2), yaw_des, x_desired[k * 12 + 11], yaw_error_max);
      if (data->_gaitScheduler->gaitData._currentGait == GaitType::STAND) {
        px_des = (1 - 0.001) * px_des + 0.001 * data->_stateEstimator->getAverageStanceFootPosWorld()(0);
        py_des = (1 - 0.001) * py_des + 0.001 * data->_stateEstimator->getAverageStanceFootPosWorld()(1);
      }
    }
    x_desired[k * 12 + 0] = px_des;
    x_desired[k * 12 + 1] = py_des;
    x_desired[k * 12 + 5] = yaw_des;
  }

  // Set the gait parameters from the Gait Scheduler
  for (int foot = 0; foot < 4; foot++) {
    gait_enabled[foot] = (double)data->_gaitScheduler->gaitData.gaitEnabled(foot);
    period_time[foot] = (double)data->_gaitScheduler->gaitData.periodTime(foot);
    time_swing[foot] = (double)data->_gaitScheduler->gaitData.timeSwing(foot);
    phase_variable[foot] = fmod((data->_gaitScheduler->gaitData.phaseVariable(foot) +
                                 dt_solve_pred / (double)data->_gaitScheduler->gaitData.periodTime(foot)), 1);
  }

  // Get the robot model parameters
  mass = data->userParameters->RPC_mass;  // [kg]
  for (int axis = 0; axis < 3; axis++) {
    inertia[axis] = data->userParameters->RPC_inertia[axis];  // [kg*m^2]
    //mass = mass;
  }

  // Get the environment parameters
  for (int axis = 0; axis < 3; axis++) {
    gravity[axis] = data->userParameters->RPC_gravity[axis];
  }
  for (int foot = 0; foot < 4; foot++) {
    mu[foot] = data->userParameters->RPC_mu;
    // Eventually get this from the vision system
    z_g[foot] = data->_stateEstimator->getStanceFootPosWorld(foot)(2); 
  }
}


/**
 * Gets the optimization variable results for all timesteps.
 */
void RPCInterface::getResult() {
  // Asynchronous filter
  if (data->userParameters->RPC_use_async_filt) {

    // Contact logic between optimization and scheduler
    contact_state_scheduled = data->_gaitScheduler->gaitData.contactStateScheduled;
    int num_stance_feet_result = 0;
    for (int foot = 0; foot < 4; foot++) {
      if (u_opt[foot * 6 + 3 + 2] == -1)  {
        // No solution has been found yet
        contact_state_result(foot) = -1;
        contact_state_last_valid(foot) = -1;
      } else if (u_opt[foot * 6 + 3 + 2] > 0.5) {
        // Foot is in contact in the optimization
        contact_state_result(foot) = 1;
        num_stance_feet_result++;
      } else  {
        // Foot is in swing in the optimization
        contact_state_result(foot) = 0;
      }
    }

    // It is a flight phase if no feet are scheduled to be in contact
    int flight_phase = 0;
    if (data->_gaitScheduler->gaitData.num_stance_feet_scheduled == 0) {
      flight_phase = 1;
    }

    // Get the resulting forces and footstep locations
    for (size_t foot = 0; foot < pat_biped::num_legs; foot++) {
      if ((contact_state_scheduled(foot) !=  data->_gaitScheduler->gaitData.contactStatePrev(foot)) ||
          iter == 0) {
        // Do not filter the first new solution
        alpha_forces = 1;
        alpha_footsteps = 1;
      } else {
        // Use the filters
        alpha_forces = data->userParameters->RPC_filter(0);
        alpha_footsteps = data->userParameters->RPC_filter(1);
      }

      // Asynchronous filter conditions
      bool normal_rpc_result = contact_state_scheduled(foot) == contact_state_result(foot);    // RPC agrees with gait schedule
      bool early_rpc_result = contact_state_scheduled(foot) == contact_state_last_valid(foot); // RPC switched from contact state and no solution returned yet
      bool late_rpc_result = contact_state_scheduled(foot) != contact_state_last_valid(foot);  // Gait switched from contact state and no solution returned yet

      // Use the optimization result with asynchronous filter compensation
      Vec3<float> p_offset(data->userParameters->Swing_step_offset(0), pow(-1, foot + 1) * data->userParameters->Swing_step_offset(1), 0);
      Vec3<float> r_hip_W = data->_stateEstimator->getResult().rBody.transpose() *
                            (data->_pat->getHipLocation(foot) + p_offset);

      // Find the steps locations and forces for the foot
      for (int axis = 0; axis < 3; axis++) {
        if (normal_rpc_result && iter != 0) {
          // Regular stance foot condition
          footFeedForwardForces(axis, foot) = (float)(u_opt[foot * 6 + 3 + axis]);
          footstepLocations(axis, foot) = (float)(p_opt[foot * 3 + axis]);

          // Save the valid result
          footFeedForwardForces_last_valid(axis, foot) = footFeedForwardForces(axis, foot);
          footstepLocations_last_valid(axis, foot) = footstepLocations(axis, foot);
          contact_state_last_valid(foot) = contact_state_result(foot);

        } else if (early_rpc_result && iter != 0) {
          // Optimized contact switch happened too early, use last valid result
          footFeedForwardForces(axis, foot) = footFeedForwardForces_last_valid(axis, foot);
          footstepLocations(axis, foot) = footstepLocations_last_valid(axis, foot);

        } else if (late_rpc_result || iter == 0) {
          // Optimized solution hasn't returned for current state, use heuristics
          footFeedForwardForces(axis, foot) = (float)(contact_state_scheduled(foot) * (-data->userParameters->RPC_mass *
                                              data->userParameters->RPC_gravity[axis] /
                                              (flight_phase + (1 - flight_phase) * data->_gaitScheduler->gaitData.num_stance_feet_scheduled)));
          if (axis == 0 || axis == 1) {
            footstepLocations(axis, foot) = (1 - contact_state_scheduled(foot)) * r_hip_W(axis);
          } else {
            footstepLocations(axis, foot) = (1 - contact_state_scheduled(foot)) * data->_stateEstimator->getStanceFootPosWorld(foot)(axis) - data->_stateEstimator->getResult().position(axis);
          }
        } else {
          printf("[RPC INTERFACE] Async Filter: Somthing is wrong\n");
        }

        // Filter the ground reaction forces
        footFeedForwardForces(axis, foot) = alpha_forces * footFeedForwardForces(axis, foot) +
                                            (1 - alpha_forces) * footFeedForwardForcesPrev(axis, foot);
        footFeedForwardForcesPrev(axis, foot) = footFeedForwardForces(axis, foot);

        // Filter the footstep locations
        footstepLocations(axis, foot) = alpha_footsteps * footstepLocations(axis, foot) +
                                        (1 - alpha_footsteps) * footstepLocationsPrev(axis, foot);
        footstepLocationsPrev(axis, foot) = footstepLocations(axis, foot);
      }
    }
  }  //  if (data->userParameters->RPC_use_async_filt)
  else {
    //pretty_print(u_opt, "u opt", 30);
    //pretty_print(p_opt, "p opt", 12);

    // Use the raw optimization result with no asynchronous filter
    for (int foot = 0; foot < 4; foot++) {
      for (int axis = 0; axis < 3; axis++) {
        footFeedForwardForces(axis, foot) = (float)(u_opt[foot * 6 + 3 + axis]);
        footstepLocations(axis, foot) = (float)(p_opt[foot * 3 + axis]);
      }
    }

  }

  iter++;
  if(!CheckSafeSolve()){
    exit(0);
  }

}

/**
 * Check the footsteps and ground reaction forces for valid solutions.
 */
bool RPCInterface::CheckSafeSolve() {
  bool safeSolve = true;

  for (size_t foot = 0; foot < pat_biped::num_legs; foot++) {
    // Modify the commands if the foot is in contact (Will be asynchronous filter eventually)
    if (data->_gaitScheduler->gaitData.contactStateScheduled(foot) == 1) {
      if (isnan(footFeedForwardForces(0, foot))) {
        cout << "[RPC INTERFACE] NaN in force solution, foot: " << foot << "  | axis: x" << endl;
        footFeedForwardForces(0, foot) = 0;
        footFeedForwardForcesPrev(0, foot) = 0;
        safeSolve = false;
      }
      if (isnan(footFeedForwardForces(1, foot))) {
        cout << "[RPC INTERFACE] NaN in force solution, foot: " << foot << "  | axis: y" << endl;
        footFeedForwardForces(1, foot) = 0;
        footFeedForwardForcesPrev(1, foot) = 0;
        safeSolve = false;
      }
      if (isnan(footFeedForwardForces(2, foot))) {
        cout << "[RPC INTERFACE] NaN in force solution, foot: " << foot << "  | axis: z" << endl;
        footFeedForwardForces(2, foot) = 0;
        footFeedForwardForcesPrev(2, foot) = 0;
        safeSolve = false;
      }
    } else {

      Vec3<float> p_offset;
      p_offset << 0, pow(-1, foot + 1) * 0.07, 0;
      Vec3<float> r_hip_W = data->_stateEstimator->getResult().rBody.transpose() *
                            (data->_pat->getHipLocation(foot) + p_offset);

      if (isnan(footstepLocations(0, foot))) {
        cout << "[RPC INTERFACE] NaN in footstep solution, foot: " << foot << "  | axis: x" << endl;
        footstepLocations(0, foot) = r_hip_W(0);
        footstepLocationsPrev(0, foot) = r_hip_W(0);
        safeSolve = false;
      }
      if (isnan(footstepLocations(1, foot))) {
        cout << "[RPC INTERFACE] NaN in footstep solution, foot: " << foot << "  | axis: y" << endl;
        footstepLocations(1, foot) = r_hip_W(1);
        footstepLocationsPrev(1, foot) = r_hip_W(1);
        safeSolve = false;
      }
      if (isnan(footstepLocations(2, foot))) {
        cout << "[RPC INTERFACE] NaN in footstep solution, foot: " << foot << "  | axis: z" << endl;
        footstepLocations(2, foot) = 0;
        footstepLocationsPrev(2, foot) = 0;
        safeSolve = false;
      }
    }
  }
  return safeSolve;
}



/**
 * Populate the RPC input data LCM and publish.
 */
void RPCInterface::SendRPCInputsLCM() {

  setRPCInputs();

  input_data.mass = mass;

  for (int i = 0; i < 3; i++) {
    input_data.inertia[i] = inertia[i];
    input_data.gravity[i] = gravity[i];
  }

  for (int i = 0; i < 4; i++) {
    input_data.gait_enabled[i] = gait_enabled[i];
    input_data.phase_variable[i] = phase_variable[i];
    input_data.period_time[i] = period_time[i];
    input_data.time_swing[i] = time_swing[i];
    input_data.mu[i] = mu[i];
    input_data.z_g[i] = z_g[i];
  }

  for (int i = 0; i < 12; i++) {
    input_data.current_state[i] = current_state[i];
    input_data.p_foot0[i] = p_foot0[i];
    input_data.r_hip[i] = r_hip[i];
    input_data.Q[i] = Q[i];
  }

  for (int i = 0; i < 24; i++) {
    input_data.R[i] = R[i];
  }

  for (int i = 0; i < 30; i++) {
    input_data.K_HX[i] = K_HX[i];
  }

  for (int i = 0; i < 72; i++) {
    input_data.x_desired[i] = x_desired[i];
  }

  if (success_test != 0) {
    lcm->publish("CONTROLLER_rpc_inputs", &input_data);
  }
}


double RPCInterface::BoundDesiredState(int k_curr, int state_num, double x_curr, double x_des_curr, double dx_curr, double error_max) {
  if (dx_curr < 0.05 && dx_curr > -0.05) {
    if (x_des_curr < x_curr) {
      x_des_curr = x_curr - std::min(abs(x_des_curr - x_curr), error_max);
    } else {
      x_des_curr = x_curr + std::min(abs(x_des_curr - x_curr), error_max);
    }
  } else {
    x_des_curr = x_curr;
    //Q[state_num] = 0;
  }

  return x_des_curr;
}


double RPCInterface::GetPredictedState(int state) {
  return X_result[state];
}


Vec3<float> RPCInterface::getFootFeedForwardForces(int leg) {
  return footFeedForwardForces.col(leg);
}


Vec3<float> RPCInterface::getFootstepLocations(int leg) {
  return footstepLocations.col(leg);
}


Mat3<float> RPCInterface::OmegaTodRPY(Vec3<float> rpy) {
  Mat3<float> Phi = Mat3<float>::Zero();
  // MIght want to add a check for gimbal lock at phi = pi/2
  Phi << cos(rpy(2)) / cos(rpy(1)), -sin(rpy(2)) / cos(rpy(1)), 0,
      sin(rpy(2)), cos(rpy(2)), 0,
      -(cos(rpy(2))*sin(rpy(1))) / cos(rpy(1)), (sin(rpy(1))*sin(rpy(2))) / cos(rpy(1)), 1;

  /*Phi << 1, sin(rpy(0))*tan(rpy(1)), cos(rpy(0))*tan(rpy(1)),
      0, cos(rpy(0)), -sin(rpy(0)),
      0, sin(rpy(0)) / cos(rpy(1)), cos(rpy(0)) / cos(rpy(1));
  */
  return Phi;
}
