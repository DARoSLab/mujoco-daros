#include "UserInputHandler.h"

// TODO: this logic could be a lot better
/*
template <typename T>
void UserInputHandler<T>::convertToStateCommands() {
  // choose the command method:
  // 1. move waypoint, 2. move robot w/ RC, 3. move robot w/ gamepad
  if (*move_waypoint > 0) {
    // Tom's Path Tracking
    // //gamepadHotkeys();
    // setAndPublishWaypoint();
    // trackPlannedPath();

    // Savva's Path Tracker
    // nothing?
    // if(update_iter>0 && update_iter%2000==0){
    //   std::cout << "[Savva Planner] Update Iter: " << update_iter << std::endl;
    // }
    if (rcCommand->jump_trigger){
      trigger_pressed = true;
    } else{
      trigger_pressed = false;
    }

  } else if (*use_rc) {
    rcCommandToStateDes();
  } else { // Gamepad
    //gamepadHotkeys();
    gamepadCommandToStateDes();
  }
  
}

template <typename T>
void UserInputHandler<T>::gamepadCommandToStateDes() {
  Vec2<float> joystickLeft, joystickRight;
  joystickLeft = gamepadCommand->leftStickAnalog;
  joystickRight = gamepadCommand->rightStickAnalog;
  joystickLeft[0] *= -1.f;
  joystickRight[0] *= -1.f;

  //  Filter joystick commands.
  leftAnalogStick = leftAnalogStick * (T(1) - filter) + joystickLeft * filter;
  rightAnalogStick = rightAnalogStick * (T(1) - filter) + joystickRight * filter;

  // Zero the state commands to start
  data.stateDes(4) = 0.0;   // pitchd
  data.stateDes(6) = 0.0;   // vxd
  data.stateDes(7) = 0.0;   // vyd
  data.stateDes(11) = 0.0;  // dyawd

  //without the shift included, may need to pull out using this
  Mat34<float> footPositions = Mat34<float>::Zero();
  for (int leg = 0; leg < 4; leg++) {
    for (int axis = 0; axis < 3; axis++) {
      footPositions(axis, leg) = (float)stateEstimator->getStanceFootPosWorld(leg)(axis);
      // footPositions(axis, leg) = (float)stateEstimator->getFutureFootPosWorld(leg)(axis);
    }
  }

  // printRawInfo();

  // Trigger Press for jumping
  if (gamepadCommand->rightTriggerButton){
    trigger_pressed = true;
  } else{
    trigger_pressed = false;
  }

  double alpha_filter = 0.008;
  pitch_filtered = (1 - alpha_filter) * pitch_filtered + alpha_filter *
                   calculateRollPitchYaw(footPositions.row(0),
                                         footPositions.row(1),
                                         footPositions.row(2),
                                         (float)stateEstimator->getResult().rpy(2))(1);

  // Desired states from the controller joysticks
  data.stateDes(4) = deadband(rightAnalogStick[1], minPitch, maxPitch) - pitch_filtered;         // pitchd
  data.stateDes(6) = deadband(leftAnalogStick[1], minVelX, maxVelX);            // vxd
  data.stateDes(7) = deadband(leftAnalogStick[0], minVelY, maxVelY);            // vyd
  data.stateDes(11) = deadband(rightAnalogStick[0], minTurnRate, maxTurnRate);  // dyawd

  commandPxdPydYawd();
  commandPzd();

  //pretty_print(data.stateDes, std::cout, "[GAME pad] DES command");
}

template <typename T>
void UserInputHandler<T>::rcCommandToStateDes() {
  Vec2<float> joystickLeft, joystickRight;
  if (rcCommand->mode == RC_mode::QP_STAND) { // Stand
    joystickLeft[0] = 0.; // Y
    joystickLeft[1] = 0.;
    joystickRight[0] = rcCommand->rpy_des[2]; // Yaw
    //height_cmd = rcCommand->height_variation;

  } else if (rcCommand->mode == RC_mode::LOCOMOTION ||
             rcCommand->mode == RC_mode::LOCOMOTIONJUMP ||
             rcCommand->mode == RC_mode::RPC_LOCOMOTION ||
             rcCommand->mode == RC_mode::VISION ||
             rcCommand->mode == RC_mode::LOCOMOTION_NET) { // Walking
    joystickLeft[0] = rcCommand->v_des[1]; // Y
    joystickLeft[1] = rcCommand->v_des[0]; // X
    joystickRight[0] = rcCommand->omega_des[2]; // Yaw
    joystickRight[1] = rcCommand->omega_des[1]; // Pitch
    //height_cmd = rcCommand->height_variation;

  } else if (rcCommand->mode == RC_mode::TWO_LEG_STANCE ||
             rcCommand->mode == RC_mode::TWO_LEG_STANCE_PRE) { // Two Contact Stand
    //joystickLeft[0] = rcCommand->p_des[1]; // Y
    joystickLeft[1] = rcCommand->p_des[0]; // X
    joystickRight[0] = rcCommand->rpy_des[2]; // Yaw
    joystickRight[1] = rcCommand->rpy_des[1]; // Pitch
    joystickLeft[0] = rcCommand->rpy_des[0]; // Roll

  } else {
    joystickLeft.setZero();
    joystickRight.setZero();
  }
  joystickLeft[0] *= -1.f;
  joystickRight[0] *= -1.f;

  leftAnalogStick = leftAnalogStick * (T(1) - filter) + joystickLeft * filter;
  rightAnalogStick = rightAnalogStick * (T(1) - filter) + joystickRight * filter;

  // Trigger Press for jumping
  if (rcCommand->jump_trigger){
    trigger_pressed = true;
  } else{
    trigger_pressed = false;
  }

  // Desired states from the controller
  data.stateDes(6) = deadband(leftAnalogStick[1], minVelX, maxVelX);  // forward linear velocity
  data.stateDes(7) = deadband(leftAnalogStick[0], minVelY, maxVelY);  // lateral linear velocity
  data.stateDes(8) = 0.0;  // vertical linear velocity

  data.stateDes(0) = dt * data.stateDes(6);  // X position
  data.stateDes(1) = dt * data.stateDes(7);  // Y position
  data.stateDes(2) = MC_BODY_HEIGHT;  // Z position height

  data.stateDes(9) = 0.0;  // Roll rate
  data.stateDes(10) = 0.0;  // Pitch rate
  data.stateDes(11) = deadband(rightAnalogStick[0], minTurnRate, maxTurnRate);  // Yaw turn rate

  data.stateDes(3) = 0.0; // Roll
  data.stateDes(4) = deadband(rightAnalogStick[1], minPitch, maxPitch);  // Pitch
  data.stateDes(5) = dt * data.stateDes(11);  // Yaw 
}

template <typename T>
void UserInputHandler<T>::commandPxdPydYawd() {
  // Get the current position and orientation in RPY
  Vec3<T> position = stateEstimator->getResult().position;
  Vec3<T> rpy = stateEstimator->getResult().rpy;

  // Simple Integration from current state
  Vec3<T> v_des_world;
  Vec3<T> v_des_robot(data.stateDes(6), data.stateDes(7), data.stateDes(8));
  v_des_world = stateEstimator->getResult().rBody.transpose() * v_des_robot;

  data.stateDes(0) = position(0) + dt * v_des_world(0);  // pxd
  data.stateDes(1) = position(1) + dt * v_des_world(1);  // pyd
  data.stateDes(5) = rpy(2) + dt * data.stateDes(11);      // yawd
}

template <typename T>
void UserInputHandler<T>::commandPzd() {
  // fixed absolute height
  data.stateDes(2) = stateEstimator->getAverageStanceFootPosWorld()(2) + MC_BODY_HEIGHT;  // pzd

}

template <typename T>
void UserInputHandler<T>::setAndPublishWaypoint(){
  Vec2<float> joystickLeft;

  if (*use_rc){
    joystickLeft[0] = rcCommand->v_des[1];
    joystickLeft[1] = rcCommand->v_des[0];
  } else {
    joystickLeft = gamepadCommand->leftStickAnalog;
    joystickLeft[0] *= -1.f;
  }
  leftAnalogStick = leftAnalogStick * (T(1) - filter) + joystickLeft * filter;

  // reset the waypoint to where the robot is if set to mode 2
  if (*move_waypoint == 2) {
    for (int i = 0; i < 2; ++i) {
      _waypoint(i) = stateEstimator->getResult().position[i];
    }
  } else {
    _waypoint(0) += 1.5*deadband(leftAnalogStick[1], minVelX, maxVelX) * dt; // maybe change the min and max speed we can move waypoint?
    _waypoint(1) += 3.0*deadband(leftAnalogStick[0], minVelY, maxVelY) * dt; // maybe change the min and max speed we can move waypoint?
    _waypoint(2) = MC_BODY_HEIGHT;
  }

  for (int i = 0; i < 3; ++i) {
    _waypointLCM.point[i] = _waypoint(i);
  }
  _waypointLCMStream.publish("WAYPOINT", &_waypointLCM);
}

template<typename T>
void UserInputHandler<T>::trackPlannedPath() {
  // without the shift included, may need to pull out using this
  Mat34<float> footPositions = Mat34<float>::Zero();
  for (int leg = 0; leg < 4; ++leg) {
    for (int axis = 0; axis < 3; ++axis) {
      footPositions(axis, leg) = (float)stateEstimator->getStanceFootPosWorld(leg)(axis);
    }
  }

  double alpha_filter = 0.008;
  pitch_filtered = (1 - alpha_filter) * pitch_filtered + alpha_filter *
                   calculateRollPitchYaw(footPositions.row(0),
                                         footPositions.row(1),
                                         footPositions.row(2),
                                         (float)stateEstimator->getResult().rpy(2))(1);

  if (*move_waypoint == 3 && !path_.empty()){ // we have path, follow it

    Vec2<double> n1_point = path_.front();

    // Current state estimate (this should come from localization sensor, not the standard state estimator)
    Vec3<T> cur_pos = stateEstimator->getResult().position;
    double cur_yaw = stateEstimator->getResult().rpy[2];

    // check if we are close enough. if yes, we remove and go to next
    const double EPS = 0.05;
    if ((abs(n1_point(0) - cur_pos(0)) <= EPS) && (abs(n1_point(1) - cur_pos(1)) <= EPS)) {
      path_.pop_front();
      if (!path_.empty())
        n1_point = path_.front();
    }
    
    // State error used for LQR tracking of path
    Eigen::Matrix<double, 6, 1> error;
    Vec2<T> pos_error_world, pos_error_body;
    pos_error_world(0) = cur_pos(0) - n1_point(0);
    pos_error_world(1) = cur_pos(1) - n1_point(1);
    pos_error_body(0) = cos(cur_yaw)*pos_error_world(0) + sin(cur_yaw)*pos_error_world(1);
    pos_error_body(1) = -sin(cur_yaw)*pos_error_world(0) + cos(cur_yaw)*pos_error_world(1);
    
    double heading_direction;
    if (path_.size() == 1){
      heading_direction = atan2(n1_point(1) - cur_pos(1), n1_point(0) - cur_pos(0));
    } else if (path_.size() == 2 ){
      Vec2<double> n2_point = path_.at(1);
      heading_direction = atan2(n2_point(1) - n1_point(1), n2_point(0) - n1_point(0));
    } else if (path_.size() > 2 ){
      Vec2<double> n2_point = path_.at(1);
      Vec2<double> n3_point = path_.at(2);
      heading_direction = atan2(n3_point(1) - n2_point(1), n3_point(0) - n2_point(0));
    } else{
      heading_direction = cur_yaw;
    }

    double yaw_error = cur_yaw - heading_direction;
    if (yaw_error > 3.14){
        yaw_error = yaw_error - 6.28;
    }else if (yaw_error < -3.14){
        yaw_error = 6.28 + yaw_error;
    }

    error(0) = pos_error_body(0);
    error(1) = pos_error_body(1);
    error(2) = yaw_error;
    error(3) = data.stateDes(6) - 0.75*PATH_x_max;
    error(4) = data.stateDes(7);
    error(5) = data.stateDes(11);

    // Control Law (from lqr)
    K << 2.6,    0.0000,   -0.0000,    0.35,    0.0000,   -0.0000,
    -0.0000,    0.58,    0.0000,    0.0000,    0,    0.0000,
    0.0000,  -0.0000,    8.4721,    0.0000,   -0.0000,    0;
    Vec3<double> u = -K*error;

    // Desired state commands in robot frame
    data.stateDes(4) = -pitch_filtered; // pitchd
    data.stateDes(6) = fmax(-PATH_x_max, fmin(PATH_x_max, u[0])); // vxd
    data.stateDes(7) = fmax(-PATH_y_max, fmin(PATH_y_max, u[1]));  // vyd
    data.stateDes(11) = fmax(-PATH_yaw_max, fmin(PATH_yaw_max, u[2]));  // dyawd
  
    // Build LCM message
    for (int i = 0; i < 2; i++){
      _dscLCM.next_point[i] = n1_point(i);
      _dscLCM.cur_pos[i] = cur_pos(i);
      _dscLCM.pos_error_world[i] = pos_error_world(i);
      _dscLCM.pos_error_body[i] = pos_error_body(i);
      _dscLCM.des_vel[i] = data.stateDes(6+i);
    } 
    _dscLCM.cur_yaw = cur_yaw;
    _dscLCM.heading_direction = heading_direction;
    _dscLCM.yaw_error = yaw_error;
    _dscLCM.des_vel[2] = data.stateDes(11);

    _waypointLCMStream.publish("DESIRED_STATE_COMMAND", &_dscLCM);

  } else {
    data.stateDes(4) = 0;   // pitchd
    data.stateDes(6) = 0;   // vxd
    data.stateDes(7) = 0;   // vyd
    data.stateDes(11) = 0;  // dyawd
  }

  commandPxdPydYawd();
  commandPzd();
}

template <typename T>
void UserInputHandler<T>::setCommandLimits(T minVelX_in, T maxVelX_in,
    T minVelY_in, T maxVelY_in, T minTurnRate_in, T maxTurnRate_in) {
  //printf("limit: %f, %f, ..\n", minVelX_in, maxVelX_in);
  minVelX = minVelX_in;
  maxVelX = maxVelX_in;
  minVelY = minVelY_in;
  maxVelY = maxVelY_in;
  minTurnRate = minTurnRate_in;
  maxTurnRate = maxTurnRate_in;
}

template <typename T>
void UserInputHandler<T>::desiredStateTrajectory(int N, Vec10<T> dtVec) {
  A = Mat12<T>::Zero();
  A(0, 0) = 1;
  A(1, 1) = 1;
  A(2, 2) = 1;
  A(3, 3) = 1;
  A(4, 4) = 1;
  A(5, 5) = 1;
  A(6, 6) = 1;
  A(7, 7) = 1;
  A(8, 8) = 1;
  A(9, 9) = 1;
  A(10, 10) = 1;
  A(11, 11) = 1;
  data.stateTrajDes.col(0) = data.stateDes;

  for (int k = 1; k < N; k++) {
    A(0, 6) = dtVec(k - 1);
    A(1, 7) = dtVec(k - 1);
    A(2, 8) = dtVec(k - 1);
    A(3, 9) = dtVec(k - 1);
    A(4, 10) = dtVec(k - 1);
    A(5, 11) = dtVec(k - 1);
    data.stateTrajDes.col(k) = A * data.stateTrajDes.col(k - 1);
    for (int i = 0; i < 12; i++) {
      // std::cout << data.stateTrajDes(i, k) << " ";
    }
  }
}

template <typename T>
float UserInputHandler<T>::deadband(float command, T minVal, T maxVal) {
  if (command < deadbandRegion && command > -deadbandRegion) {
    return 0.0;
  } else {
    return (command / (2)) * (maxVal - minVal);
  }
}

template <typename T>
void UserInputHandler<T>::printRawInfo() {
  // Increment printing iteration
  printIter++;

  // Print at requested frequency
  if (printIter == printNum) {
    std::cout << "[DESIRED STATE COMMAND] Printing Raw Gamepad Info...\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Button Start: " << gamepadCommand->start
              << " | Back: " << gamepadCommand->back << "\n";
    std::cout << "Button A: " << gamepadCommand->a
              << " | B: " << gamepadCommand->b << " | X: " << gamepadCommand->x
              << " | Y: " << gamepadCommand->y << "\n";
    std::cout << "Left Stick Button: " << gamepadCommand->leftStickButton
              << " | X: " << gamepadCommand->leftStickAnalog[0]
              << " | Y: " << gamepadCommand->leftStickAnalog[1] << "\n";
    std::cout << "Right Analog Button: " << gamepadCommand->rightStickButton
              << " | X: " << gamepadCommand->rightStickAnalog[0]
              << " | Y: " << gamepadCommand->rightStickAnalog[1] << "\n";
    std::cout << "Left Bumper: " << gamepadCommand->leftBumper
              << " | Trigger Switch: " << gamepadCommand->leftTriggerButton
              << " | Trigger Value: " << gamepadCommand->leftTriggerAnalog
              << "\n";
    std::cout << "Right Bumper: " << gamepadCommand->rightBumper
              << " | Trigger Switch: " << gamepadCommand->rightTriggerButton
              << " | Trigger Value: " << gamepadCommand->rightTriggerAnalog
              << "\n\n";
    std::cout << std::endl;

    // Reset iteration counter
    printIter = 0;
  }
}

template <typename T>
void UserInputHandler<T>::printStateCommandInfo() {
  // Increment printing iteration
  printIter++;

  // Print at requested frequency
  if (printIter == printNum) {
    std::cout << "[DESIRED STATE COMMAND] Printing State Command Info...\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Position X: " << data.stateDes(0)
              << " | Y: " << data.stateDes(1) << " | Z: " << data.stateDes(2)
              << "\n";
    std::cout << "Orientation Roll: " << data.stateDes(3)
              << " | Pitch: " << data.stateDes(4)
              << " | Yaw: " << data.stateDes(5) << "\n";
    std::cout << "Velocity X: " << data.stateDes(6)
              << " | Y: " << data.stateDes(7) << " | Z: " << data.stateDes(8)
              << "\n";
    std::cout << "Angular Velocity X: " << data.stateDes(9)
              << " | Y: " << data.stateDes(10) << " | Z: " << data.stateDes(11)
              << "\n";
    std::cout << std::endl;
    std::cout << std::endl;

    // Reset iteration counter
    printIter = 0;
  }
}

*/

template class UserInputHandler<double>;
template class UserInputHandler<float>;
