#include "cMPC_BipedLocomotion.h"
#include <iostream>
#include <Utilities/Timer.h>
#include <pretty_print.h>
#include <ParamHandler/ParamHandler.hpp>

#define ORIGINAL_IMPLEMENTATION
#define DRAW_DEBUG_SWINGS
////////////////////
// Controller
////////////////////

cMPC_BipedLocomotion::cMPC_BipedLocomotion(
    float _dt, int _iterations_between_mpc,PatParameters* parameters,
    const FloatingBaseModel<float> * model) :
  _model(model),
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(20),
  dt(_dt),
  standing(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(20, 20, 20, 20),"Standing"),
  walking(horizonLength, Vec4<int>(0,0, 10, 10), Vec4<int>(10, 10, 10, 10),"Walking")
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
  for(int i = 0; i < _num_cp; i++) firstSwing[i] = true;
  firstRun = true;
  iterationCounter = 0;
}


void cMPC_BipedLocomotion::_SetupCommand(ControlFSMData<float> & data){
  float x_vel_cmd, y_vel_cmd;

  _roll_turn_rate = 0.;
  _pitch_turn_rate = 0.;
  _yaw_turn_rate = 0.;
  _x_vel_des = 0.;
  _y_vel_des = 0.;

  _roll_des = 0.;
  _pitch_des = 0.;
}

void cMPC_BipedLocomotion::run(ControlFSMData<float>& data) {
  // Command Setup
  _SetupCommand(data);
  // updateModel(data);
  gaitNumber = 0;  // 0: Standing, 1: Walking

  // pick gait
  gait = &walking;
  current_gait = gaitNumber;
  gait->setIterations(iterationsBetweenMPC, iterationCounter);

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0];

  pFoot[0] = _model->_pGC[pat_biped_linkID::RF];
  pFoot[1] = _model->_pGC[pat_biped_linkID::LF];

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
    for(int f=0; f< _num_cp; f++){

      init_hip_loc[f] = seResult.position + seResult.rBody.transpose()*data._pat->getHipLocation(f);
      init_foot_loc[f] = pFoot[f];
    }

    for(int i = 0; i < _num_cp; i++)
    {

      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    firstRun = false;
  }

  // foot placement
  for(int l = 0; l < _num_cp; l++) swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for(int i = 0; i < _num_cp; i++)
  {
    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }

    footSwingTrajectories[i].setHeight(0.05); // sehwan - larger swing height for running

    Vec3<float> pRobotFrame;

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

    Vec3<float> stand_foot_pos = pFoot[(i+1)%2];
    // Vec3<float> body_from_foot = seResult.position - stand_foot_pos;
    //float p_rel_max = 0.35f;
    float p_rel_max = 0.2f;

    // Assume that v_des_robot is in the robot frame
    Vec2<float> pf_rel_body, pf_rel_world;
    Mat2<float> rotMatYaw;
    rotMatYaw << cos(seResult.rpy[2]), -sin(seResult.rpy[2]), sin(seResult.rpy[2]), cos(seResult.rpy[2]);

    Vec2<float> swingStates = gait->getSwingState();
    // Vec2<float> contactStates = gait->getContactState();

    // if (iterationCounter==2000) {
    //   exit(0);
    // }

    int swing_leg = swingStates[1]>0.0 && swingStates[1]<1.0? 1 : 0;
    // pretty_print(swingStates, std::cout, "Swing States:");
    // pretty_print(contactStates, std::cout, "Contact States: ");
    // printf("[%d] i: %d Swing Leg: %d \n", iterationCounter, i,  swing_leg);

    if(swing_leg==i){
      if(swingTimeRemaining[i]<0.5*_swing_time)
        _enable_fp_planning[i] = true;

      if(firstSwing[i]){
        firstSwing[i] = false;
        footSwingTrajectories[i].setInitialPosition(pFoot[i]);
        liftSwingTrajectories[i].setInitialPosition(pFoot[i]);

        Vec3<float> p_mid;
        p_mid = seResult.position;
        p_mid[0] += _default_foot_loc[0];
        p_mid[1] += pow(-1, i+1)*_default_foot_loc[1];
        p_mid[2] = _default_foot_loc[2] + _swing_height; //= seResult.position + seResult.rBody.transpose()*pRobotFrame; //current hip
        // p_mid += (init_foot_loc[i] - init_hip_loc[i]);
        printf("Body Height %f \n", _body_height);
        // p_mid[2] = 0.05;
        liftSwingTrajectories[i].setMiddlePosition(p_mid);
        p_mid[2] = 0.0;
        //just for drawing
        liftSwingTrajectories[i].setFinalPosition(p_mid);
        std::cout << "####################################################" << '\n';
        _enable_fp_planning[i] = false;
        _done_fp_planning[i] = false;
      }
      // else if(swingTimeRemaining[i]<0.33/2.0){//start planning at the beginning of swing
      else if(_enable_fp_planning[i] && !_done_fp_planning[i]){//start planning at the beginning of swing
        std::cout << "####################################################" << '\n';
        printf("updating foot step \n");
        _done_fp_planning[i] = true;
        OutputReversalPL pl_output;
        ParamReversalPL pl_param;
        Vec2<float> des_location_;
        // Vec3<float> com_pos, com_vel;
        com_pos = seResult.position + _model->getComPosWorld(); // world frame
        com_vel = _model->getComVel();

        pl_param.swing_time = swingTimeRemaining[i]; //0.33/2;
        pl_param.des_loc = des_location_;
        pl_param.stance_foot_loc = stand_foot_pos;
        pl_param.b_positive_sidestep = swing_leg == 1;

        des_location_<< com_pos[0]+0.2, com_pos[1] - 0.1;
        // des_location_<< 0.0, 0.0;


        Vec3<float> target_loc;
        //body position + com relative to body frame expressed in world frame
        //getComPosWorld = w_R_b*com_b
        // com_pos = seResult.position + _model->getComPosWorld(); // world frame
        // _planner->CheckEigenValues(swingTimeRemaining[i]);// print eigen values
        _planner->getNextFootLocation(com_pos, // + stand_foot_pos,
                                      com_vel,
                                      target_loc,
                                      &pl_param,
                                      &pl_output);


        // target_loc -= stand_foot_pos;

        target_loc[2] = 0;

        for(int xy = 0; xy < 2; xy++){
          Pf[xy] = target_loc[xy];
        }
        // Pf[2] = -0.002;
        Pf[2] = 0.0;
        footSwingTrajectories[i].setFinalPosition(Pf);
        liftSwingTrajectories[i].setMiddlePosition(pFoot[i]);
        liftSwingTrajectories[i].setFinalPosition(Pf);
        for (int j=0; j<3; j++){
          pNextFoot[i][j] = Pf[j];
        }
      }
      else{
          //do nothing
          // printf("foot: %d t: %f\n", i, swingTimeRemaining[i]);
      }
    }

  }
  // gait
  Vec2<float> contactStates = gait->getContactState();
  Vec2<float> swingStates = gait->getSwingState();

  int* mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, data);
  // std::cout << "contact states " << contactStates << '\n';
  Vec2<float> se_contactState; se_contactState.setZero();


  // calc gait
  // iterationCounter++;

  for(size_t foot(0); foot < pat_biped::num_legs; ++foot)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    // if(swingState > 0) // foot is in swing
    int swing_leg = swingStates[1]>0.0 && swingStates[1]<1.0? 1 : 0;
    if(swing_leg == (int)foot) // foot is in swing
    {
      // if(firstSwing[foot])
      // {
      //   firstSwing[foot] = false;
      //   footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      // }
      #ifdef DRAW_DEBUG_SWINGS
            // auto* debugPath = data.visualizationData->addPath();
            // if(debugPath) {
            //   debugPath->num_points = 100;
            //   debugPath->color = {0.2,1,0.2,0.5};
            //   float step = (1.f - swingState) / 100.f;
            //   for(int i = 0; i < 100; i++) {
            //     footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
            //     debugPath->position[i] = footSwingTrajectories[foot].getPosition();
            //   }
            // }
            // auto* finalSphere = data.visualizationData->addSphere();
            // if(finalSphere) {
            //   finalSphere->position = footSwingTrajectories[foot].getPosition();
            //   finalSphere->radius = 0.02;
            //   finalSphere->color = {0.6, 0.6, 0.2, 0.7};
            // }
            // footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
            // auto* actualSphere = data.visualizationData->addSphere();
            // auto* goalSphere = data.visualizationData->addSphere();
            // goalSphere->position = footSwingTrajectories[foot].getPosition();
            // actualSphere->position = pFoot[foot];
            // goalSphere->radius = 0.02;
            // actualSphere->radius = 0.02;
            // goalSphere->color = {0.2, 1, 0.2, 0.7};
            // actualSphere->color = {0.8, 0.2, 0.2, 0.7};

            auto* debugPath = data.visualizationData->addPath();
            if(debugPath) {
              debugPath->num_points = 100;
              debugPath->color = {1.0, 0.2, 0.2, 0.5};
              float step = (1.f - swingState) / 100.f;
              // float step = 0.01f;
              for(int i = 0; i < 100; i++) {
                // printf("[%d] swingState: %f step: %f phase: %f swingTimes %f\n",
                //         i, swingState, step, swingState + i * step, swingTimes[foot]);
                // printf("[%d] swingState: %f step: %f phase: %f swingTimes %f\n",
                //         i, swingState, step, i * step, swingTimes[foot]);
                // liftSwingTrajectories[foot].computeLiftSwingTrajectory(swingState + i * step, 0.5, swingTimes[foot]);
                liftSwingTrajectories[foot].computeLiftSwingTrajectory(swingState + i * step, 0.5, swingTimes[foot]);

                debugPath->position[i] = liftSwingTrajectories[foot].getPosition();
              }
            }



          auto* currSphere = data.visualizationData->addSphere();
          currSphere ->position = pFoot[foot];
          currSphere ->radius = 0.05;
          currSphere ->color = {0.0, 0.0, 0.8, 0.5};

          liftSwingTrajectories[foot].computeLiftSwingTrajectory(0.4999, 0.5, 0.31/2);
          auto* midSphere = data.visualizationData->addSphere();
          midSphere ->position = liftSwingTrajectories[foot].getPosition();
          midSphere ->radius = 0.05;
          midSphere ->color = {0.8, 0.0, 0.0, 0.5};

          liftSwingTrajectories[foot].computeLiftSwingTrajectory(0.9999, 0.5, 0.31/2);
          auto* finalSphere = data.visualizationData->addSphere();
          finalSphere ->position = liftSwingTrajectories[foot].getPosition();
          finalSphere ->radius = 0.05;
          finalSphere ->color = {0.0, 0.8, 0.0, 0.5};
          //
          auto* CoMSphere = data.visualizationData->addSphere();
          CoMSphere->position = com_pos;
          CoMSphere->position[2] = 0.0;
          CoMSphere->radius = 0.05;
          CoMSphere->color = {0.0, 0.0, 0.0, 0.5};

          // liftSwingTrajectories[leg].computeLiftSwingTrajectory(1.0, alpha, swingTimes[leg]);
          // auto* finalSphere = data.visualizationData->addSphere();
          // finalSphere ->position = liftSwingTrajectories[leg].getPosition();;
          // finalSphere ->radius = 0.05;
          // finalSphere ->color = {0.0, 0.8, 0.0, 0.5};
      #endif

      // footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      liftSwingTrajectories[foot].computeLiftSwingTrajectory(swingState, 0.5, swingTimes[foot]);

      // Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();

      Vec3<float> pDesFootWorld = liftSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = liftSwingTrajectories[foot].getVelocity();

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      // aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      aFoot_des[foot] = liftSwingTrajectories[foot].getAcceleration();

      if(!data.userParameters->use_wbc){
        Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._pat->getHipLocation(foot);
        Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;

        data._legController->commands[foot].kpJoint.setZero();
        data._legController->commands[foot].kdJoint.setZero();
        //pretty_print(Kp, std::cout, "Kp");
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
      se_contactState[foot] = contactState;
      if(!data.userParameters->use_wbc){

        Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
        Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();

        Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._pat->getHipLocation(foot);
        Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;
      }
    }
  }
  //data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  _updateWBC_CMD();
  iterationCounter++;
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

void cMPC_BipedLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data) {
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);


    if(current_gait == 0) // Standing
    {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des,
        (float)stand_traj[5],
        (float)stand_traj[0],
        (float)stand_traj[1],
        (float)_body_height,
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

void cMPC_BipedLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult();

  //float Q[12] = {0.75, 0.25, 10, 2, 2, 30, 0, 0, 0.3, 0.2, 0.2, 0.2};


  // For walking & running
  float Q[12] = {3.25, 2.25, 1, 0.0, 0, 30, 1, 0.1, 0.1, 0.2, 0.2, 0.1};
  //float Q[12] = {3.25, 2.25, 3, 0.1, 1, 1, 0, 0, 0.1, 0.2, 0.2, 0.1};
  //float Q[12] = {10.25, 4.25, 10, 2, 2, 40, 0., 0., 0.1, 5.2, 5.2, 0.1}; // Running (not working)
  //float Q[12] = {7., 3., 10, 1, 15, 35, 0.0, 0., 0.1, 4., 10., 0.1}; // Running (not working)

  // Stand up
  //float Q[12] = {2.25, 2.45, 8, 250, 250, 250, 0.1, 0.1, 0.2, 0.2, 0.2, 0.1};

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH

  float* p = seResult.position.data();
  //Vec3<float> comPosWorld =  seResult.rBody.transpose()*_model->getComPos();
  //for(int i(0); i<3; ++i) p[i] += _model->getComPos()[i];  // com posn in world frame (?)
  //for(int i(0); i<3; ++i) p[i] += comPosWorld[i];
  float* v = seResult.vWorld.data(); // body velocity
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  // gait
  Vec2<float> swingStates = gait->getSwingState();

  float r[6];

  for (int i=0;i<6;i++){
    float swingState = swingStates[i%2];
    if(swingState > 0) // foot is in swing
    {
      r[i] = pNextFoot[i%2][i/2] - p[i/2];
    }
    else{
      r[i] = pFoot[i%2][i/2]  - p[i/2];
    }
  }

#ifdef ORIGINAL_IMPLEMENTATION // pass in current footsteps to mpc solver
  for (int i=0;i<6;i++){
    r[i] = pFoot[i%2][i/2]  - p[i/2];
  }
#endif

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  // printf("body position: %f, %f, %f\n", p[0], p[1], p[2]);

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

  for(int cp = 0; cp < _num_cp; cp++)
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


  // debugging: calculate state state_trajectory

  Eigen::Matrix<float,13,1> x_state; // current state
  Eigen::Matrix<float,13,1> x_next; // next predicted state
  Eigen::Matrix<float,13,13> A_c = get_A(); // A matrix in state space
  Eigen::Matrix<float,13,12> B_c = get_B(); // B matrix in state space

  x_state << 0, 0, yaw, p[0], p[1], p[2], w[0], w[1], w[2], v[0], v[1], v[2], -9.81;
  x_next = A_c*x_state + B_c*u_input;
}

void cMPC_BipedLocomotion::cMPCParameterInitialization(const std::string & file){
  ParamHandler handler(file + ".yaml");

  handler.getValue("swing_time", _swing_time);
  handler.getValue("swing_height", _swing_height);
  handler.getValue("body_height", _body_height);
  handler.getVector("default_foot_location", _default_foot_loc);
  printf("[cMPC_biped Planner] Parameter Setup is completed\n");
}
