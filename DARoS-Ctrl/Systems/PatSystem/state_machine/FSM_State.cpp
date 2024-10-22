/*============================= FSM State =============================*/
/**
 * FSM State base class
 */

#include "FSM_State.h"

#ifdef MACHINE_LEARNING_BUILD
  #include <RPC/RPCNetInterface.h>
#endif

/**
 * Constructor for the FSM State class.
 *
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name
 * @param stateStringIn the string name of the current FSM state
 */
template <typename T>
FSM_State<T>::FSM_State(ControlFSMData<T> *_controlFSMData,
    FSM_StateName stateNameIn, std::string stateStringIn): 
  _data(_controlFSMData),
  stateName(stateNameIn),
  stateString(stateStringIn) {

    std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn << std::endl;

    if (_data->_pat) {
      ZeroLegControllerCommands();
      // Foot locations 
      footstepLocations = D3Mat<T>::Zero(3, _data->_pat->NUM_FEET);  // Footstep locations for next step
      footPositionsCurr = D3Mat<T>::Zero(3, _data->_pat->NUM_FEET);  // Footstep locations at the current stance location
      footPositionsTD = D3Mat<T>::Zero(3, _data->_pat->NUM_FEET);    // Footstep locations at touchdown
      //
      // Create the RPC interface instance
      switch ((int)_data->userParameters->RPC_interface_type) {
        case K_THREADED:
          RPC_Interface = ThreadedRPCInterface::GetInstance(_data);
          break;

        case K_LCM:
          RPC_Interface = LCMRPCInterface::GetInstance(_data);
          break;

        default:
          // Nothing chosen (No RPC)
          break;
      }
    }
  }


/* =================== Robot Utilities =================== */

/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param qDes desired joint position
 * @param dqDes desired joint velocity
 */
template <typename T>
void FSM_State<T>::jointPDControl(
    int leg, Vec3<T> qDes, Vec3<T> qdDes, T kp, T kd) {

  kpMat << kp, 0, 0, 0, kp, 0, 0, 0, kp;
  kdMat << kd, 0, 0, 0, kd, 0, 0, 0, kd;


  _data->_legController->commands[leg].kpJoint = kpMat;
  _data->_legController->commands[leg].kdJoint = kdMat;

  _data->_legController->commands[leg].tauFeedForward.setZero();
  _data->_legController->commands[leg].qDes = qDes;
  _data->_legController->commands[leg].qdDes = qdDes;
}


/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param pDes desired foot position
 * @param vDes desired foot velocity
 * @param kp_cartesian P gains
 * @param kd_cartesian D gains
 */
template <typename T>
void FSM_State<T>::cartesianImpedanceControl(int leg, Vec3<T> pDes,
    Vec3<T> vDes,
    Vec3<double> kp_cartesian,
    Vec3<double> kd_cartesian) {
  _data->_legController->commands[leg].pDes = pDes;
  // Create the cartesian P gain matrix
  kpMatAll[leg] << kp_cartesian[0], 0, 0,
    0, kp_cartesian[1], 0,
    0, 0, kp_cartesian[2];
  _data->_legController->commands[leg].kpCartesian = kpMatAll[leg];

  _data->_legController->commands[leg].vDes = vDes;
  // Create the cartesian D gain matrix
  kdMatAll[leg] << kd_cartesian[0], 0, 0,
    0, kd_cartesian[1], 0,
    0, 0, kd_cartesian[2];
  _data->_legController->commands[leg].kdCartesian = kdMatAll[leg];
}


/**
 * Get the position of the foot in the world frame from the leg controller data.
 */
template <typename T>
Vec3<T> FSM_State<T>::GetFootPositionWorld(int leg) {
  return _data->_stateEstimator->getResult().position
    + (_data->_stateEstimator->getResult().rBody.transpose()
        * (_data->_pat->getHipLocation(leg)
          + _data->_legController->datas[leg]->p));
}

/**
 * Get the position of the foot in the world frame from the leg controller data.
 */
template <typename T>
Vec3<T> FSM_State<T>::GetAverageFootPositionWorld() {
  Vec3<T> averageFootPosWorld = {0, 0, 0};
  for (int foot = 0; foot < _data->_pat->NUM_FEET; foot++) {
    averageFootPosWorld = averageFootPosWorld + footPositionsCurr.col(foot);
  }
  return averageFootPosWorld / _data->_pat->NUM_FEET;
}


/**
 * Convert the position from the world frame to the hip frame
 */
template <typename T>
Vec3<T> FSM_State<T>::transformPositionWorldToHip(int leg, Vec3<T> positionWorld) {
  return _data->_stateEstimator->getResult().rBody
    * (positionWorld
        - _data->_stateEstimator->getResult().position)
    - _data->_pat->getHipLocation(leg);
}

/**
 * Convert the position from the world frame to the hip frame
 */
template <typename T>
Vec3<T> FSM_State<T>::VelocityWorldToHip(int leg, Vec3<T> velocityWorld) {
  return _data->_stateEstimator->getResult().rBody
    * (velocityWorld
        - _data->_stateEstimator->getResult().vWorld);
}


/**
 * Convert the position from the world frame to the hip frame. Negative to
 * match the convention in the leg controller.
 */
template <typename T>
Vec3<T> FSM_State<T>::ForceWorldToHip(Vec3<T> forceWorld) {
  return -_data->_stateEstimator->getResult().rBody * forceWorld;
}


/**
 * Stance leg logic for impedance control. Prevent leg slipping and
 * bouncing, as well as tracking the foot velocity during high speeds.
 */
template <typename T>
void FSM_State<T>::StanceLegImpedanceControl(int leg) {
  // Impedance control for the stance leg
  cartesianImpedanceControl(
      leg, footPositionsCurr.col(leg), Vec3<T>::Zero(),
      _data->userParameters->stand_kp_cartesian,
      _data->userParameters->stand_kd_cartesian);
}


/**
 *
 */
template <typename T>
Vec3<T> FSM_State<T>::FootstepWorldPosition(int leg, Vec3<T> footstepLocation) {
  return _data->_stateEstimator->getResult().position
    + footstepLocation
    + _data->_stateEstimator->getResult().vWorld * _data->_gaitScheduler->gaitData.timeSwingRemaining(leg);
}


/**
 * Sets the footswing trajectories in the world frame.
 */
template <typename T>
void FSM_State<T>::SetFootswingTrajectories(int leg, Vec3<T> initialPos, Vec3<T> finalPos, T height_des) {
  footSwingTrajectories[leg].setHeight(height_des);
  footSwingTrajectories[leg].setInitialPosition(initialPos);
  footSwingTrajectories[leg].setFinalPosition(finalPos);
  footSwingTrajectories[leg].computeSwingTrajectoryBezier(
      _data->_gaitScheduler->gaitData.phaseSwing(leg),
      _data->_gaitScheduler->gaitData.timeSwing(leg));
}


/**
 * Pass the locally stored leg controller command values to the leg controllers.
 */
template <typename T>
void FSM_State<T>::SetLegControllerCommands() {
  for (int foot = 0; foot < _data->_pat->NUM_FEET; foot++) {
    _data->_legController->commands[foot].forceFeedForward = footFeedForwardForces.col(foot);
    _data->_legController->commands[foot].pDes = footPositions.col(foot);
    _data->_legController->commands[foot].vDes = footVelocities.col(foot);
    _data->_legController->commands[foot].kpCartesian = kpMatAll[foot];
    _data->_legController->commands[foot].kdCartesian = kdMatAll[foot];
  }
}


/**
 * Zero all of the locally stored leg controller commands.
 */
template <typename T>
void FSM_State<T>::ZeroLegControllerCommands() {
  jointFeedForwardTorques = D3Mat<T>::Zero(3, _data->_pat->NUM_FEET); // feed forward joint torques
  jointPositions = D3Mat<T>::Zero(3, _data->_pat->NUM_FEET);          // joint angle positions
  jointVelocities = D3Mat<T>::Zero(3, _data->_pat->NUM_FEET);         // joint angular velocities
  footFeedForwardForces = D3Mat<T>::Zero(3, _data->_pat->NUM_FEET);   // feedforward forces at the feet
  footPositions = D3Mat<T>::Zero(3, _data->_pat->NUM_FEET);           // cartesian foot positions
  footVelocities = D3Mat<T>::Zero(3, _data->_pat->NUM_FEET);          // cartesian foot velocities
  kpMat = Mat3<T>::Zero();                                                  // cartesian P gain matrix
  kdMat = Mat3<T>::Zero();                                                  // cartesian D gain matrix
}


/* =================== Gait Scheduler Modifications =================== */


/**
 * Modifies the gait scheduler naturally to stand based on robot desired and actual velocity.
 * Naturally returns to stance in push recovery or if velocity commands aren't present.
 *
 * TODO: push and command detection in transition to stance
 */
template <typename T>
void FSM_State<T>::NaturalGaitModification() {
  // Only use the natural gait modification if selected
  if (_data->userParameters->gait_override == 3 || _data->userParameters->gait_override == 4) {
    // Get the actual and desired velocity norms
    Vec3<T> pWorld = _data->_stateEstimator->getResult().position;
    Vec3<T> pFootAveWorld = _data->_stateEstimator->getAverageStanceFootPosWorld();
    Vec3<T> vWorld = _data->_stateEstimator->getResult().vWorld;

    pError_act_norm_filt = alpha_ngm * pError_act_norm_filt + (1 - alpha_ngm) * sqrt(pow(pFootAveWorld(0) - pWorld(0), 2) + pow(pFootAveWorld(1) - pWorld(1), 2));
    v_act_norm_filt = alpha_ngm * v_act_norm_filt + (1 - alpha_ngm) * sqrt(pow(vWorld(0), 2) + pow(vWorld(1), 2));
    v_des_norm_filt = alpha_ngm * v_des_norm_filt + (1 - alpha_ngm) * sqrt(pow(_data->_desiredStateCommand->data.stateDes[6], 2) + pow(_data->_desiredStateCommand->data.stateDes[7], 2));

    // Get the actual and desired turning rates (approximated by omega body Z)
    yaw_rate_act_filt = alpha_ngm * yaw_rate_act_filt + (1 - alpha_ngm) * fabs(_data->_stateEstimator->getResult().omegaBody(2));
    yaw_rate_des_filt = alpha_ngm * yaw_rate_des_filt + (1 - alpha_ngm) * fabs(_data->_desiredStateCommand->data.stateDes[11]);

    // Conditions to enter locomotion
    bool locomotion_command = (v_des_norm_filt > 0.01 || yaw_rate_des_filt > 0.01);
    bool vel_push = (v_act_norm_filt > _data->userParameters->gait_disturbance(1) || yaw_rate_act_filt > _data->userParameters->gait_disturbance(2));
    bool pos_push = (pError_act_norm_filt > _data->userParameters->gait_disturbance(0));

    // Conditions to enter stance
    bool stance_command = !locomotion_command;
    bool vel_recovery = v_act_norm_filt < _data->userParameters->gait_recovery(1) && yaw_rate_act_filt < _data->userParameters->gait_recovery(2);
    bool pos_recovery = (pError_act_norm_filt < _data->userParameters->gait_recovery(0));

    //std::cout << pError_act_norm << std::endl;


    // Case structure for natural gait selection... add filter to the norms and turning rates
    switch (naturalGaitState) {
      case NaturalGaitState::STANCE:
        if (locomotion_command) {  // make these settings for thresholds
          // Commanded velocity has occured, start trotting
          naturalGaitState = NaturalGaitState::NORMAL_LOCOMOTION;
          _data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
          cout << "[FSM STATE] Transition from STANCE to NORMAL_LOCOMOTION" << endl;

        } else if (vel_push || pos_push) {
          // Push has occurred, start a trotting gait
          naturalGaitState = NaturalGaitState::DISTURBANCE;
          _data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
          cout << "[FSM STATE] Transition from STANCE to DISTURBANCE | pos: " << pos_push << " | vel: " << vel_push << endl;

        } else {
          // Continue normal stance
        }
        break;

      case NaturalGaitState::DISTURBANCE:
        // Currently in push recovery state
        if (locomotion_command) {
          naturalGaitState = NaturalGaitState::NORMAL_LOCOMOTION;
          recoveryIter = 0;
          cout << "[FSM STATE] Transition from DISTURBANCE to NORMAL_LOCOMOTION" << endl;

        } else if (stance_command && vel_recovery && pos_recovery) {
          // Conditions met to transition to stance
          recoveryIter++;
          if (recoveryIter >= (0.25 / _data->userParameters->controller_dt)) {
            // If the robot is in a good state for a certain amount of time, begin transition
            naturalGaitState = NaturalGaitState::TRANSITIONING_TO_STANCE;
            _data->_gaitScheduler->gaitData._nextGait = GaitType::TRANSITION_TO_STAND;
            recoveryIter = 0;
            cout << "[FSM STATE] Transition from DISTURBANCE to TRANSITIONING_TO_STANCE" << endl;
          }

        } else {
          // Continue recovering from push
          recoveryIter = 0;
        }
        break;

      case NaturalGaitState::NORMAL_LOCOMOTION:
        // Currently in normal locomotion
        if (stance_command && vel_recovery && pos_recovery) {
          // Conditions met to start transition to stance
          recoveryIter++;
          if (recoveryIter >= (0.25 / _data->userParameters->controller_dt)) {
            naturalGaitState = NaturalGaitState::TRANSITIONING_TO_STANCE;
            _data->_gaitScheduler->gaitData._nextGait = GaitType::TRANSITION_TO_STAND;
            recoveryIter = 0;
            cout << "[FSM STATE] Transition from NORMAL_LOCOMOTION to TRANSITIONING_TO_STANCE" << endl;
          }

        } else {
          // Continue normal locomotion
          if (_data->userParameters->gait_override == 4) {
            // Modify stance time to adjust for desired velocity
            T stance_time_natural = (2.0 * 0.26 *
                tan(_data->userParameters->gait_max_leg_angle * 3.1415926 / 180.0) / std::max(v_des_norm_filt, (T)0.0001));
            _data->_gaitScheduler->period_time_natural = 
              std::max(std::min(swing_time_natural + stance_time_natural, 
                    swing_time_natural + (T)_data->userParameters->gait_max_stance_time), 
                  swing_time_natural + (T)_data->userParameters->gait_min_stance_time);
            _data->_gaitScheduler->switching_phase_natural = 
              1.0 - (swing_time_natural / _data->_gaitScheduler->period_time_natural);
          }
          recoveryIter = 0;
        }
        break;

      case NaturalGaitState::TRANSITIONING_TO_STANCE:
        // Currently transitioning to stance
        if (locomotion_command) {
          // Commanded velocity has occured, start trotting
          naturalGaitState = NaturalGaitState::NORMAL_LOCOMOTION;
          _data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
          gaitTransitionIter = 0;
          cout << "[FSM STATE] Transition from TRANSITIONING_TO_STANCE to NORMAL_LOCOMOTION" << endl;

        } else if (vel_push || pos_push) {
          // Push has occured, continue trotting
          naturalGaitState = NaturalGaitState::DISTURBANCE;
          _data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
          gaitTransitionIter = 0;
          cout << "[FSM STATE] Transition from TRANSITIONING_TO_STANCE to DISTURBANCE | pos: " << pos_push << " | vel: " << vel_push << endl;

        } else {
          // Continue transitioning to stance
          gaitTransitionIter++;
          if (gaitTransitionIter >= (0.5 / _data->userParameters->controller_dt)) {
            naturalGaitState = NaturalGaitState::STANCE;
            _data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;
            recoveryIter = 0;
            gaitTransitionIter = 0;
            cout << "[FSM STATE] Transition from TRANSITIONING_TO_STANCE to STANCE" << endl;
          }
        }
        break;
    }
  }
}


/* =================== Controller Logic =================== */

/**
 * Runs the Whole-Body Control
 *   - Currently only tested with FSM_State_LocomotionRPC
 */
template<typename T>
void FSM_State<T>::runWholeBodyController() {

  // WBC
  Vec3<T> v_des_world;
  double yaw_rate_des(0.);

  if (_data->userParameters->use_rc) {
    const rc_control_settings* rc_cmd = _data->_desiredStateCommand->rcCommand;
    Vec3<T> v_des_robot(
        rc_cmd->v_des[0], // x vel
        rc_cmd->v_des[1] * 0.6, // y vel
        0.);
    v_des_world = _data->_stateEstimator->getResult().rBody.transpose() * v_des_robot;
    yaw_rate_des = rc_cmd->omega_des[2];
  } else {
    Vec3<T> v_des_robot(
        _data->_desiredStateCommand->data.stateDes[6],
        _data->_desiredStateCommand->data.stateDes[7], 0);
    v_des_world = _data->_stateEstimator->getResult().rBody.transpose() * v_des_robot;
    yaw_rate_des = _data->_desiredStateCommand->data.stateDes[11];
  }

  _wbc_data->vBody_des[0] = v_des_world(0);
  _wbc_data->vBody_des[1] = v_des_world(1);
  _wbc_data->vBody_des[2] = 0.;

  T dt = _data->userParameters->controller_dt;
  Vec3<T> curr_pos = _data->_stateEstimator->getResult().position;
  _world_des_wbc[0] = curr_pos[0] + _wbc_data->vBody_des[0] * dt;
  _world_des_wbc[1] = curr_pos[1] + _wbc_data->vBody_des[1] * dt;
  _world_des_wbc[2] = _data->_desiredStateCommand->data.stateDes[2];

  _wbc_data->pBody_des = _world_des_wbc;

  _wbc_data->aBody_des.setZero();

  _wbc_data->pBody_RPY_des[0] = 0.;
  _wbc_data->pBody_RPY_des[1] = _data->_desiredStateCommand->data.stateDes[4];
  _wbc_data->pBody_RPY_des[2] = RPC_Interface->x_desired[5];//_data->_desiredStateCommand->data.stateDes[5];

  _wbc_data->vBody_Ori_des[0] = 0;
  _wbc_data->vBody_Ori_des[1] = 0;
  _wbc_data->vBody_Ori_des[2] = yaw_rate_des;

  for (int foot = 0; foot < _data->_pat->NUM_FEET; ++foot) {
    _wbc_data->pFoot_des[foot] = footSwingTrajectories[foot].getPosition();
    _wbc_data->vFoot_des[foot] = footSwingTrajectories[foot].getVelocity();
    _wbc_data->aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

    _wbc_data->Fr_des[foot] = RPC_Interface->getFootFeedForwardForces(foot);
    _wbc_data->contact_state[foot] =
      _data->_gaitScheduler->gaitData.contactStateScheduled(foot);
  }
  _wbc_ctrl->run(_wbc_data, *_data);

  //pretty_print(_wbc_data->contact_state, std::cout, "wbc contact state");
  //for (int foot = 0; foot < _data->_pat->NUM_FEET; ++foot) {
    //_data->_legController->commands[foot].pDes = footPositions.col(foot);
    //_data->_legController->commands[foot].vDes = footVelocities.col(foot);
    //_data->_legController->commands[foot].kpCartesian = kpMatAll[foot];
    //_data->_legController->commands[foot].kdCartesian = kdMatAll[foot];

    //if (!_data->_gaitScheduler->gaitData.gaitEnabled(foot)) {
      //_data->_legController->commands[foot].tauFeedForward << 0.0, 0.0, 0.0;
      //_data->_legController->commands[foot].kpJoint << 0.0, 0.0, 0.0,
        //0.0, 0.0, 0.0,
        //0.0, 0.0, 0.0;
      //_data->_legController->commands[foot].kdJoint << 0.0, 0.0, 0.0,
        //0.0, 0.0, 0.0,
        //0.0, 0.0, 0.0;
    //}
  //}
}


/**
 * Runs the Regularized Predictive Controller
 */
template<typename T>
void FSM_State<T>::runRegularizedPredictiveController() {
  // Initialize the local leg controller commands to zero
  jointFeedForwardTorques = Mat34<T>::Zero();                // feed forward joint torques
  jointPositions = Mat34<T>::Zero();   // joint angle positions
  jointVelocities = Mat34<T>::Zero();  // joint angular velocities
  footFeedForwardForces = Mat34<T>::Zero();              // feedforward forces at the feet
  footPositions = Mat34<T>::Zero();  // cartesian foot positions
  footVelocities = Mat34<T>::Zero();

  // Initialize the state estimator contact state to 0
  Vec4<T> se_contactState(0, 0, 0, 0);

  // Initialize the cartesian position gains
  Kp_swing << _data->userParameters->Swing_Kp_cartesian(0), 0, 0,
           0, _data->userParameters->Swing_Kp_cartesian(1), 0,
           0, 0, _data->userParameters->Swing_Kp_cartesian(2);
  Kd_swing << _data->userParameters->Swing_Kd_cartesian(0), 0, 0,
           0, _data->userParameters->Swing_Kd_cartesian(1), 0,
           0, 0, _data->userParameters->Swing_Kd_cartesian(2);

  // Modify Gait Schedule based on desired inputs
  NaturalGaitModification();

  // Get the results from the latest RPC optimization
  RPC_Interface->getResult();
  
  // Parse commands into correct leg controller data
  for (int leg = 0; leg < _data->_pat->NUM_FEET; leg++) {
    kpMatAll[leg] = Mat3<T>::Zero();
    kdMatAll[leg] = Mat3<T>::Zero();
    if (_data->_gaitScheduler->gaitData.gaitEnabled(leg)) {
      if (_data->_gaitScheduler->gaitData.contactStateScheduled(leg)) {
        // Foot is in contact
        if (_data->_gaitScheduler->gaitData.touchdownScheduled(leg)) {
          // Any behavior to happen during shcheduled touchdown
        }

        // Get the feedforward forces from the RPC
        footFeedForwardForces.col(leg) = ForceWorldToHip(
            RPC_Interface->getFootFeedForwardForces(leg));

        // Get the current foot position in the world frame
        footPositionsCurr.col(leg) = _data->_stateEstimator->getFootPosWorld(leg);

        // Set the desired foot positions and velocities in the hip frame  |
        // TODO: Make this into a funciton and take into account robot velocity
        // and hip turning velocity
        footPositions.col(leg) =
          transformPositionWorldToHip(leg, footPositionsTD.col(leg));
        footVelocities.col(leg) =
          VelocityWorldToHip(leg, Vec3<T>::Zero());

        // Set the cartesian impedance gains to stance values
        kpMatAll[leg] << 0 * Kp_stance;
        kdMatAll[leg] << Kd_stance;

      } else {
        // Foot is not in contact
        if (_data->_gaitScheduler->gaitData.liftoffScheduled(leg)) {
          // Any behavior to happen during shcheduled liftoff
        }

        // Get the resulting footstep location from the RPC in the world frame
        footstepLocations.col(leg) =
          FootstepWorldPosition(leg, RPC_Interface->getFootstepLocations(leg));
        footstepLocations.col(leg)(2) = 
          footstepLocations.col(leg)(2) + _data->userParameters->Swing_step_offset(2);  // Get ground height from vision at (px, py)

        // Set the footswing trajectory
        SetFootswingTrajectories(leg, footPositionsCurr.col(leg),
            footstepLocations.col(leg),
            _data->userParameters->Swing_traj_height);

        // Set the desired foot positions and velocities in the hip frame 
        // TODO: Make this a function and take into account the hip turning velocity
        footPositions.col(leg) =
          transformPositionWorldToHip(leg, footSwingTrajectories[leg].getPosition());
        footVelocities.col(leg) =
          VelocityWorldToHip(leg, footSwingTrajectories[leg].getVelocity());

        // Set the cartesian impedance gains to swing values
        kpMatAll[leg] << Kp_swing;
        kdMatAll[leg] << Kd_swing;

        // Set joint D control for stability
        _data->_legController->commands[leg].kdJoint = _data->userParameters->Swing_Kd_joint(0) * Mat3<float>::Identity();
        _data->_legController->commands[leg].tauFeedForward[2] = 
          _data->userParameters->Swing_use_tau_ff *
          50 * (_data->_legController->datas[leg]->q(2) < .1) *
          exp(1. / (0.01 + fabs(_data->_legController->datas[leg]->q(2))));

        footPositionsTD.col(leg) = _data->_stateEstimator->getFootPosWorld(leg);
      }

      // Set the current leg contact phase for the state estimator
      se_contactState[leg] = _data->_gaitScheduler->gaitData.phaseStance(leg);

    } else {
      // Leg is not enabled

      // Set the cartesian impedance gains to swing values
      kpMatAll[leg] << Kp_swing;
      kdMatAll[leg] << Kd_swing;

      // Set the desired foot positions and velocities in the hip frame  | TODO: Make this a function and take into account the hip turning velocity
      footPositions.col(leg) << 0.0, 0.0, -0.15;
      footVelocities.col(leg) << 0.0, 0.0, 0.0;

      // Disabled leg is not used for state estimation
      se_contactState[leg] = 0.0;

    }
  }

  // Notify the state estimator of the contact states
  _data->_stateEstimator->setContactPhase(se_contactState);

  //printf("FSM STATE result START ==============\n");
  //for(size_t leg(0); leg<4; ++leg){
    //printf("leg: %zu\n", leg);
    //pretty_print(RPC_Interface->getFootFeedForwardForces(leg), std::cout, "foot feed forward");
    //pretty_print(RPC_Interface->getFootstepLocations(leg), std::cout, "foot step location");
  //}
  //pretty_print(se_contactState, std::cout, "contact state");

  //_data->_desiredStateCommand->printStateCommandInfo();
  //pretty_print(_data->_desiredStateCommand->data.stateDes, std::cout, "[FSM State] DES command");
  //printf("in STATE\n");
  //_data->_desiredStateCommand->data.print();
  //std::cout<<_data<<std::endl;
  //std::cout<<_data->_desiredStateCommand<<std::endl;
  //std::cout<<&_data->_desiredStateCommand->data<<std::endl;
  //printf("in STATE END\n");
  //printf("FSM STATE result END ==============\n");
}


/* =================== Safety Functions =================== */


/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
template <typename T>
void FSM_State<T>::turnOnAllSafetyChecks() {
  // Pre controls safety checks
  checkSafeOrientation = true;  // check roll and pitch

  // Post control safety checks
  checkPDesFoot = true;          // do not command footsetps too far
  checkForceFeedForward = true;  // do not command huge forces
  checkLegSingularity = true;    // do not let leg
}


/**
 *
 */
template <typename T>
void FSM_State<T>::turnOffAllSafetyChecks() {
  // Pre controls safety checks
  checkSafeOrientation = false;  // check roll and pitch

  // Post control safety checks
  checkPDesFoot = false;          // do not command footsetps too far
  checkForceFeedForward = false;  // do not command huge forces
  checkLegSingularity = false;    // do not let leg
}


template<typename T>
bool FSM_State<T>::locomotionSafe() {
  auto &seResult = _data->_stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;

  if (std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  // if (std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
  //   printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
  //   return false;
  // }

  for (int leg = 0; leg < 4; leg++) {
    auto p_leg = _data->_legController->datas[leg]->p;
    if (p_leg[2] > 0) {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if (std::fabs(p_leg[1] > 0.18)) {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    auto v_leg = _data->_legController->datas[leg]->v.norm();
    if (std::fabs(v_leg) > 9.) {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }
  return true;
}



/* =================== Visualization =================== */

/**
 * Visualize the current swing leg trajectory.
 */
template <typename T>
void FSM_State<T>::VisualizeSwingLegTrajectory() {
  for (int foot = 0; foot < _data->_pat->NUM_FEET; foot++) {
    if (_data->_gaitScheduler->gaitData.contactStateScheduled(foot) == 0) {
      // Add the swing trajectories
      //auto* swingTraj = _data->visualizationData->addPath();
      PathVisualization* swingTraj = _data->visualizationData->addPath();
      if(swingTraj){
        swingTraj->num_points = 100;
        swingTraj->color = {legColors[3 * foot + 0], legColors[3 * foot + 1], legColors[3 * foot + 2], 0.5};
        T step = (1.f - _data->_gaitScheduler->gaitData.phaseSwing(foot)) / 100.f;
        for (int i = 0; i < 100; i++) {
          footSwingTrajectories[foot].computeSwingTrajectoryBezier(
              _data->_gaitScheduler->gaitData.phaseSwing(foot) + i * step,
              _data->_gaitScheduler->gaitData.timeSwing(foot));
          swingTraj->position[i] = footSwingTrajectories[foot].getPosition();
        }
      }
    }
  }
}


/**
 * Visualize an input force arrow.
 */
template <typename T>
void FSM_State<T>::VisualizeInputForce(Vec3<T> position, Vec3<T> direction, T color[3], T brightness) {
  // Add an arrow for the force at timestep k
  ArrowVisualization* forceArrow = _data->visualizationData->addArrow();
  T forceArrowScale = 300.f;
  forceArrow->base_position[0] = position(0);
  forceArrow->base_position[1] = position(1);
  forceArrow->base_position[2] = position(2);
  forceArrow->direction[0] = direction(0) / forceArrowScale;
  forceArrow->direction[1] = direction(1) / forceArrowScale;
  forceArrow->direction[2] = direction(2) / forceArrowScale;
  forceArrow->head_width = 0.0125;
  forceArrow->head_length = 0.04;
  forceArrow->shaft_width = 0.004;
  forceArrow->color = {brightness*color[0], brightness*color[1], brightness*color[2], 0.7};
}


/**
 * Visualize a step position shpere.
 */
template <typename T>
void FSM_State<T>::VisualizeStepPos(Vec3<T> position, T color[3], T brightness) {
  // Add a sphere for the step location at timestep k
  SphereVisualization* footstepSphere = _data->visualizationData->addSphere();
  footstepSphere->position[0] = position(0);
  footstepSphere->position[1] = position(1);
  footstepSphere->position[2] = position(2);
  footstepSphere->radius = 0.02;
  footstepSphere->color = {brightness*color[0], brightness*color[1], brightness*color[2], 0.7};
}

/**
 * Visualize an input arrow.
 */
template <typename T>
void FSM_State<T>::VisualizeArrow(Vec3<T> position, Vec3<T> direction, T color[3], T brightness, T scale) {
  ArrowVisualization* forceArrow = _data->visualizationData->addArrow();
  forceArrow->base_position[0] = position(0);
  forceArrow->base_position[1] = position(1);
  forceArrow->base_position[2] = position(2);
  forceArrow->direction[0] = direction(0) / scale;
  forceArrow->direction[1] = direction(1) / scale;
  forceArrow->direction[2] = direction(2) / scale;
  forceArrow->head_width = 0.0125;
  forceArrow->head_length = 0.04;
  forceArrow->shaft_width = 0.004;
  forceArrow->color = {brightness*color[0], brightness*color[1], brightness*color[2], 0.7};
}


/**
 * Visualize a shpere.
 */
template <typename T>
void FSM_State<T>::VisualizeSphere(Vec3<T> position, T color[3], T brightness) {
  SphereVisualization* footstepSphere = _data->visualizationData->addSphere();
  footstepSphere->position[0] = position(0);
  footstepSphere->position[1] = position(1);
  footstepSphere->position[2] = position(2);
  footstepSphere->radius = 0.04;
  footstepSphere->color = {brightness*color[0], brightness*color[1], brightness*color[2], 0.8};
}

/**
 * Visualize a box.
 */
template <typename T>
void FSM_State<T>::VisualizeBox(Vec3<T> position, Vec3<T> dimension, Vec3<T> rpy, T color[3], T brightness) {
  BlockVisualization* box = _data->visualizationData->addBlock();
  box->corner_position[0] = position(0);
  box->corner_position[1] = position(1);
  box->corner_position[2] = position(2);
  box->dimension[0] = dimension(0);
  box->dimension[1] = dimension(1);
  box->dimension[2] = dimension(2);
  box->rpy[0] = 180./3.1416*rpy(0); // rpy in degrees
  box->rpy[1] = 180./3.1416*rpy(1);
  box->rpy[2] = 180./3.1416*rpy(2);
  box->color = {brightness*color[0], brightness*color[1], brightness*color[2], 0.35};
}

/**
 * Visualize the predicted results for the states and inputs as well as the swing
 * leg trajectories.
 */
template <typename T>
void FSM_State<T>::VisualizePredictionRPC() {
  if (_data->userParameters->RPC_visualize_pred) {
    PathVisualization* trajectoryPred = _data->visualizationData->addPath();
    PathVisualization* trajectoryDes = _data->visualizationData->addPath();
    if (trajectoryPred) {
      trajectoryPred->num_points = 5;
      trajectoryPred->color = {0.2, 0.2, 0.7, 0.5};
      trajectoryDes->num_points = 5;
      trajectoryDes->color = {0.2, 0.7, 0.1, 0.5};

      double yaw_des_viz = 0;
      Vec3<T> stepPos;
      Vec3<T> inputForce;

      // Iterate for all predicted timesteps
      for (int k = 0; k < 5; k++) {
        // Adapt the brightness of the colors to darken further along prediction horizon
        T brightness = (1 - k * 1.f / 6.f);

        // The predicted COM trajectory positions
        trajectoryPred->position[k][0] = RPC_Interface->X_result[36 * k + 0];
        trajectoryPred->position[k][1] = RPC_Interface->X_result[36 * k + 1];
        trajectoryPred->position[k][2] = RPC_Interface->X_result[36 * k + 2];

        // The desired trajectory
        if (k == 0) {
          trajectoryDes->position[k][0] = RPC_Interface->x_desired[k * 12 + 0];
          trajectoryDes->position[k][1] = RPC_Interface->x_desired[k * 12 + 1];
          trajectoryDes->position[k][2] = RPC_Interface->x_desired[k * 12 + 2];
          yaw_des_viz = RPC_Interface->x_desired[k * 12 + 5];
        } else {
          yaw_des_viz = yaw_des_viz + RPC_Interface->dt_pred[k - 1] *
            RPC_Interface->x_desired[(k - 1)  * 12 + 11];

          trajectoryDes->position[k][0] = trajectoryDes->position[k - 1][0] + RPC_Interface->dt_pred[k - 1] *
            (RPC_Interface->x_desired[(k - 1)  * 12 + 6] * std::cos(yaw_des_viz) -
             RPC_Interface->x_desired[(k - 1)  * 12 + 7] * std::sin(yaw_des_viz));
          trajectoryDes->position[k][1] = trajectoryDes->position[k - 1][1] + RPC_Interface->dt_pred[k - 1] *
            (RPC_Interface->x_desired[(k - 1)  * 12 + 7] * std::cos(yaw_des_viz) +
             RPC_Interface->x_desired[(k - 1)  * 12 + 6] * std::sin(yaw_des_viz));
          trajectoryDes->position[k][2] = trajectoryDes->position[k - 1][2] + RPC_Interface->dt_pred[k - 1] * RPC_Interface->input_data.x_desired[k * 12 + 8];
        }

        // COM position at prediction timestep k
        SphereVisualization* positionCoM = _data->visualizationData->addSphere();
        positionCoM->radius = 0.01;
        positionCoM->position = trajectoryPred->position[k];
        positionCoM->color = {brightness * 1.0f, brightness * 0.2f, brightness * 0.2f, 0.5};

        // COM des position at prediction timestep k
        SphereVisualization* positionCoMDes = _data->visualizationData->addSphere();
        positionCoMDes->radius = 0.01;
        positionCoMDes->position = trajectoryDes->position[k];
        positionCoMDes->color = {brightness * 0.2f, brightness * 1.0f, brightness * 0.2f, 0.5};

        for (int foot = 0; foot < _data->_pat->NUM_FEET; foot++) {
          // Draw step location if swing foot
          if (RPC_Interface->X_result[36 * k + 12 + 6 * foot + 5] != 0) {
            // Get predicted step position
            stepPos << RPC_Interface->X_result[36 * k + 0] + RPC_Interface->X_result[36 * k + 12 + 6 * foot + 0],
            RPC_Interface->X_result[36 * k + 1] + RPC_Interface->X_result[36 * k + 12 + 6 * foot + 1],
            RPC_Interface->X_result[36 * k + 2] + RPC_Interface->X_result[36 * k + 12 + 6 * foot + 2];

            // Get predicted input force
            inputForce << RPC_Interface->X_result[36 * k + 12 + 6 * foot + 3],
            RPC_Interface->X_result[36 * k + 12 + 6 * foot + 4],
            RPC_Interface->X_result[36 * k + 12 + 6 * foot + 5];

            // Add a sphere for the step location at timestep k
            VisualizeStepPos(stepPos, legColors + 3 * foot, brightness);

            // Add an arrow for the force at timestep k
            VisualizeInputForce(stepPos, inputForce, legColors + 3 * foot, brightness);
          }
        }
      }
    }
  }
}


//template class FSM_State<double>;
template class FSM_State<float>;
