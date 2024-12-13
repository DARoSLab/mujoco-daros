#include "cMPC_BipedLocomotion.h"
#include <iostream>
#include <utilities/Timer.h>
#include <utilities/pretty_print.h>

#define ORIGINAL_IMPLEMENTATION

////////////////////
// Controller
////////////////////

cMPC_BipedLocomotion::cMPC_BipedLocomotion(
    float _dt, int _iterations_between_mpc, HumanoidParameters* parameters,
    const FloatingBaseModel<float> * model) :
  _model(model),
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(20),
  dt(_dt),
  standing(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(20,20,20,20),"Standing"),
  walking(horizonLength, Vec4<int>(0,0,10,10), Vec4<int>(10,10,10,10),"Walking"),
  running(horizonLength, Vec4<int>(0,0,10,10),Vec4<int>(9, 9, 9, 9),"Running") 
  // sehwan - shorter stances for running
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.3, 800);

  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < _num_cp; i++)
    firstSwing[i] = true;

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
}

void cMPC_BipedLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
  iterationCounter = 0;
}


void cMPC_BipedLocomotion::_SetupCommand(ControlFSMData_Humanoid<float> & data){

  float x_vel_cmd, y_vel_cmd;

  if(data.userParameters->use_rc){
    const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand;
    data.userParameters->cmpc_gait = rc_cmd->variable[0];
    _body_height += rc_cmd->height_variation * 0.08;
  }

  _roll_turn_rate = -data._desiredStateCommand->data.stateDes(9);
  _pitch_turn_rate = -5.*data._desiredStateCommand->data.stateDes(4);
  _yaw_turn_rate = -data._desiredStateCommand->data.stateDes(11);
  _x_vel_des = 0.5*data._desiredStateCommand->data.stateDes(6);
  _y_vel_des = 0.1*data._desiredStateCommand->data.stateDes(7);

  //printf("x vel des: %f\n", _x_vel_des);
  //printf("y vel des: %f\n", _y_vel_des);

  _roll_des = data._stateEstimator->getResult().rpy[0] + dt * _roll_turn_rate;
  _pitch_des = data._stateEstimator->getResult().rpy[1] + dt * _pitch_turn_rate;
  //_yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _yaw_des = _yaw_des + dt * _yaw_turn_rate;
}

void cMPC_BipedLocomotion::run(ControlFSMData_Humanoid<float>& data) {
  // Command Setup
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;
  auto& seResult = data._stateEstimator->getResult();

  // Check if transition to standing
  if(((gaitNumber == 0) && current_gait != 0) || firstRun)
  {
    stand_traj[0] = seResult.position[0] + _model->getComPos()[0];
    stand_traj[1] = seResult.position[1] + _model->getComPos()[1];
    stand_traj[2] = _body_height;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  gait = &standing;
  if(gaitNumber == 1){
    gait = &walking;
  }
  else if(gaitNumber == 2){
    gait = &running;
    _body_height = 0.48;
  }
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //pretty_print(v_des_world, std::cout, "v des world");

  //Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(_roll_des - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0];

  pFoot[0] = _model->_pGC[humanoid_link::R_toe];
  pFoot[1] = _model->_pGC[humanoid_link::R_heel];

  pFoot[2] = _model->_pGC[humanoid_link::L_toe];
  pFoot[3] = _model->_pGC[humanoid_link::L_heel];

  //pretty_print(right_toe, std::cout, "right toe");
  //pretty_print(right_heel, std::cout, "right heel");
  //pretty_print(left_toe, std::cout, "left toe");
  //pretty_print(left_heel, std::cout, "left heel");

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0] + _model->getComPos()[0];
    world_position_desired[1] = seResult.position[1] + _model->getComPos()[1];
    world_position_desired[2] = seResult.rpy[2];

    for(int i = 0; i < 4; i++)
    {

      footSwingTrajectories[i].setHeight(0.08);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    firstRun = false;
  }

  // foot placement
  for(int l = 0; l < 4; l++) swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for(int i = 0; i < 4; i++)
  {
    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    //if(swingTimeRemaining[i] < swingTimes[i]*0.9){
      //continue;
    //}

    footSwingTrajectories[i].setHeight(0.08); // sehwan - larger swing height for running

    Vec3<float> pRobotFrame;
    if(i<2) pRobotFrame = (data._humanoid->getHipLocation(0));
    else pRobotFrame = (data._humanoid->getHipLocation(1));

    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected =
      coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    //printf("desired velocity: %f, %f, %f\n", _x_vel_des, _y_vel_des, _yaw_turn_rate);

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
          + des_vel * swingTimeRemaining[i]);

    //float p_rel_max = 0.35f;
    float p_rel_max = 0.25f;

    // Assume that v_des_robot is in the robot frame
    Vec2<float> pf_rel_body, pf_rel_world;
    Mat2<float> rotMatYaw;
    rotMatYaw << cos(seResult.rpy[2]), -sin(seResult.rpy[2]), sin(seResult.rpy[2]), cos(seResult.rpy[2]);

    pf_rel_body[0] =
      seResult.vBody[0] * .5 * stance_time +
      .33f*(seResult.vBody[0]-v_des_robot[0]) +
      0.5f*sqrt(seResult.position[2]/9.81f) * (seResult.vBody[1]*_yaw_turn_rate);

    pf_rel_body[1] =

      seResult.vBody[1] * .5 * stance_time +
      .33f*(seResult.vBody[1]-v_des_robot[1])
    + 0.5f*sqrt(seResult.position[2]/9.81f) * (-seResult.vBody[0]*_yaw_turn_rate);

    pf_rel_world = rotMatYaw * pf_rel_body;
    for(int xy = 0; xy < 2; xy++){
      pf_rel_world[xy] = fminf(fmaxf(pf_rel_world[xy], -p_rel_max), p_rel_max);
      Pf[xy] += pf_rel_world[xy];
    }
    //Pf[2] = -0.003;
    Pf[2] = 0.018;
    footSwingTrajectories[i].setFinalPosition(Pf);

    for (int j=0; j<3; j++){
      pNextFoot[i][j] = Pf[j];
    }

  }

  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, data);

  Vec4<float> se_contactState(0,0,0,0);


  // calc gait
  iterationCounter++;

  for(int foot(0); foot < 4; ++foot)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
      se_contactState[foot] = contactState;
    }
  }
  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  _updateWBC_CMD();
}

void cMPC_BipedLocomotion::_updateWBC_CMD(){
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  contact_state = gait->getContactState();
}

void cMPC_BipedLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData_Humanoid<float> &data) {
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);


    if(current_gait == 0) // Standing
    {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
        (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
        (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
        (float)_body_height/*fsm->main_control_settings.p_des[2]*/,
        _roll_turn_rate,
        _pitch_turn_rate,
        _yaw_turn_rate,
        0,0,0};

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];


        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 0] = trajAll[12 * (i - 1) + 0] + dtMPC * _roll_turn_rate;
          trajAll[12*i + 1] = trajAll[12 * (i - 1) + 1] + dtMPC * _pitch_turn_rate;
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    else {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = {(float)rpy_comp[0],  // 0
        (float)rpy_comp[1],    // 1
        _yaw_des,    // 2
        //yawStart,    // 2
        xStart,                                   // 3
        yStart,                                   // 4
        //0.0,                                   // 4
        (float)_body_height,      // 5
        0,                                        // 6
        0,                                        // 7
        _yaw_turn_rate,  // 8
        v_des_world[0],                           // 9
        v_des_world[1],                           // 10
        0};                                       // 11

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    Timer solveTimer;

    solveDenseMPC(mpcTable, data);
    //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

}

void cMPC_BipedLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData_Humanoid<float> &data) {
  auto seResult = data._stateEstimator->getResult();

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};


  // For walking & running
  //float Q[12] = {3.25, 2.25, 10, 0.3, 2, 30, 0, 0, 0.1, 5.2, 5.2, 0.1};
  //float Q[12] = {3.25, 2.25, 10, 0.3, 1, 30, 0, 0, 0.1, 5.2, 5.2, 0.1};
  //float Q[12] = {10.25, 4.25, 10, 2, 2, 40, 0., 0., 0.1, 5.2, 5.2, 0.1}; // Running (not working)
  float Q[12] = {7., 3., 10, 1, 15, 35, 0.0, 0., 0.1, 4., 10., 0.1}; // Running (not working)

  // Stand up
  //float Q[12] = {2.25, 2.45, 8, 250, 250, 250, 0.1, 0.1, 0.2, 0.2, 0.2, 0.1};

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH

  float* p = seResult.position.data();
  Vec3<float> comPosWorld =  seResult.rBody.transpose()*_model->getComPos();
  //for(int i(0); i<3; ++i) p[i] += _model->getComPos()[i];  // com posn in world frame (?)
  for(int i(0); i<3; ++i) p[i] += comPosWorld[i];
  float* v = seResult.vWorld.data(); // body velocity
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  // gait
  //Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();

  float r[12];

  for (int i=0;i<12;i++){
    float swingState = swingStates[i%4];
    if(swingState > 0) // foot is in swing
    {
      r[i] = pNextFoot[i%4][i/4] - p[i/4];
    }
    else{
      r[i] = pFoot[i%4][i/4]  - p[i/4];
    }
  }

#ifdef ORIGINAL_IMPLEMENTATION // pass in current footsteps to mpc solver
  for (int i=0;i<12;i++){
    r[i] = pFoot[i%4][i/4]  - p[i/4];
  }
#endif

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC,horizonLength,0.4,800);

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
      _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  //t1.stopPrint("Setup MPC");

  Timer t2;
  //cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);
  //t2.stopPrint("Run MPC");
  //printf("MPC Solve time %f ms\n", t2.getMs());

 Eigen::Matrix<float,12,1> u_input;

  for(int cp = 0; cp < 4; cp++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++){
      f[axis] = get_solution(cp*3 + axis);
      u_input[3*cp + axis] = f[axis];
    }
    //printf("[%d] %7.3f %7.3f %7.3f\n", cp, f[0], f[1], f[2]);

    f_ff[cp] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[cp] = f;
    //pretty_print(Fr_des[cp], std::cout, "FR des");
  }


  // ***************************************************************************************************************
  // debugging: calculate state state_trajectory

  Eigen::Matrix<float,13,1> x_state; // current state
  Eigen::Matrix<float,13,1> x_next; // next predicted state
  Eigen::Matrix<float,13,13> A_c = get_A(); // A matrix in state space
  Eigen::Matrix<float,13,12> B_c = get_B(); // B matrix in state space

  x_state << 0, 0, yaw, p[0], p[1], p[2], w[0], w[1], w[2], v[0], v[1], v[2], -9.81;
  x_next = A_c*x_state + B_c*u_input;

}
