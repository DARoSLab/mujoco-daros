#include "WBC_BipedLocomotion.h"
#include <iostream>
#include <Utilities/Timer.h>
#include <pretty_print.h>
#include <ParamHandler/ParamHandler.hpp>

#define ORIGINAL_IMPLEMENTATION
// #define DRAW_DEBUG_SWINGS
////////////////////
// Controller
////////////////////

WBC_BipedLocomotion::WBC_BipedLocomotion(PatParameters* parameters,
    const FloatingBaseModel<float> * model):
  _model(model),
  standing(20, Vec2<int>(0,0), Vec2<int>(20,20),"Standing"),
  walking(20, Vec2<int>(10,0), Vec2<int>(10,10),"Walking"),
  walkingDS(20, Vec2<int>(9, 0), Vec2<int>(11,11),"WalkingDS"),
  walkingDS2(21, Vec2<int>(10, 0), Vec2<int>(11,11),"WalkingDS2"),
  running(20, Vec2<int>(0,10), Vec2<int>(9, 9),"Running"),
  _tvrLCM(getLcmUrl(255))
{
  _parameters = parameters;
  dt =  _parameters->controller_dt;
  tvrParameterInitialization(THIS_COM"config/pat_locomotion_parameters");
  _planner = new Reversal_LIPM_Planner();
  _planner->PlannerInitialization(THIS_COM"config/tvr_planner");
  _planner->setOmega(_body_height);//body height

  // sgait = new SimpleGait(_swing_time, 2.0/9, dt);
  for(int i = 0; i < _num_cp; i++)
    firstSwing[i] = true;

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
  // _com_vel.setZero();
  // _com_pos.setZero();

}

void WBC_BipedLocomotion::initialize(){
  for(int i = 0; i < _num_cp; i++)
    firstSwing[i] = true;
  firstRun = true;
  _iter = 0;
}


void WBC_BipedLocomotion::_setupCommand(ControlFSMData<float> & data){

  _x_vel_des = 0.0;
  _y_vel_des = 0.0;
  _x_com_des += 0.5*_x_vel_des*dt;
  _y_com_des += 0.5*_y_vel_des*dt;


}

void WBC_BipedLocomotion::run(ControlFSMData<float>& data) {
  // Command Setup
  _setupCommand(data);
  auto& seResult = data._stateEstimator->getResult();

  pFoot[0] = _model->_pGC[pat_biped_linkID::RF];
  pFoot[1] = _model->_pGC[pat_biped_linkID::LF];
  if(_enable_double_stance>0.0)
    gait = &walkingDS;
  else
    gait = &walking;
  int iter_per_div = (int)(19*_swing_time/(9*20*dt));
  // std::cout << "iter_per_div: " << iter_per_div << '\n';
  // std::cout << "iter_per_div: " << _swing_time/(9*dt) << '\n';
  gait->setIterations(iter_per_div, _iter);
  // sgait->updateGait();
  // std::cout << "cs: " << sgait->getContactState()<< '\n';
  // auto cs  = sgait->getContactState();
  // auto ss  = sgait->getSwingState();
  // _gait_lcm.contact_state[0] = cs[0];
  // _gait_lcm.contact_state[1] = cs[1];
  // _gait_lcm.swing_state[0] = ss[0];
  // _gait_lcm.swing_state[1] = ss[1];
  // _gait_lcm.swing_time_remaining[0] = sgait->getRemainingSwingTime(0);
  // _gait_lcm.swing_time_remaining[1] = sgait->getRemainingSwingTime(1);
  // _gait_lcm.phase = sgait->getPhase();
  // if(ss[0]>0.5 && ss[1]>0.5){
  //   std::cout << "Fight phase" << '\n';
  //   exit(0);
  // }
  // _gaitLCM.publish("pat_gait", &_gait_lcm);
  // if(_iter % 1000==0)
  //

  // some first time initialization
  if(firstRun)
  {
    firstRun = false;
  }

  // foot placement
  for(int l = 0; l < _num_cp; l++){
    swingTimes[l] = gait->getCurrentSwingTime(_swing_time/9, l);
  }
  Vec3<float> Pf;


  _com_pos = _model->_state.bodyPosition + _model->getComPosWorld(); // world frame
  _com_vel = _model->getComVel();

  // static float t = 0.0;
  // t  = fmod(t+dt, _swing_time);
  // if(_step_count>0){
  //
  //   _planner->getDesiredCOMState(
  //     t,
  //     _com_pos_ini,  _com_vel_ini,
  //     _stance_foot_loc_ini,
  //     _com_pos_state, _com_vel_state);
  //
  // }


  for(int k(0); k<2; k++){
    _tvr_lcm.COM_pos[k] = _com_pos[k];
    _tvr_lcm.COM_vel[k] = _com_vel[k];
    // _tvr_lcm.COM_pos_des[k] = _com_pos_state[k];
    // _tvr_lcm.COM_vel_des[k] = _com_vel_state[k];
  }
  // _tvr_lcm.COM_pos[2] = _com_pos[2];
  // _tvr_lcm.COM_vel[2] = _com_vel[2];
  // _tvr_lcm.step_count = _step_count;
  // _tvrLCM.publish("tvr_data_lcmt", &_tvr_lcm);


  for(int i = 0; i < _num_cp; i++)
  {
    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    // swingTimeRemaining[i] = sgait->getRemainingSwingTime(i);

    Vec3<float> stand_foot_pos = pFoot[(i+1)%2];
    Vec2<float> swingStates = gait->getSwingState();
    // Vec2<float> swingStates = sgait->getSwingState();

    int swing_leg = swingStates[1]>0.0 && swingStates[1]<1.0? 1 : 0;


    // if(swing_leg==i){
    if(swingStates[i]>0.0){
      if(swingTimeRemaining[i]<0.5*_swing_time)
        _enable_fp_planning[i] = true;

      if(firstSwing[i]){
        firstSwing[i] = false;
        liftSwingTrajectories[i].setInitialPosition(pFoot[i]);

        Vec3<float> p_mid;
        // p_mid = seResult.position;
        p_mid = _model->_state.bodyPosition; //correct
        // p_mid << _com_pos; //test
        p_mid[0] += _default_foot_loc[0];
        p_mid[1] += pow(-1, i+1)*_default_foot_loc[1];
        p_mid[2] = _default_foot_loc[2] + _swing_height; //= seResult.position + seResult.rBody.transpose()*pRobotFrame; //current hip

        liftSwingTrajectories[i].setMiddlePosition(p_mid);
        p_mid[2] = 0.0;
        //just for drawing
        liftSwingTrajectories[i].setFinalPosition(p_mid);
        _enable_fp_planning[i] = false;
        _done_fp_planning[i] = false;
        for(int k(0); k<3; k++){
          _tvr_lcm.foot_landing_loc[k] = pFoot[(i+1)%2][k];
        }
      }
      // else if(swingTimeRemaining[i]<0.33/2.0){//start planning at the beginning of swing
      else if(_enable_fp_planning[i]){

        if(!_done_fp_planning[i]){//start planning at the beginning of swing
          // std::cout << "####################################################" << '\n';
          // printf("updating foot step \n");
          // std::cout << "####################################################" << '\n';
          _done_fp_planning[i] = true;
          _step_count++;
          OutputReversalPL pl_output;
          ParamReversalPL pl_param;
          Vec2<float> des_location_;

          // _com_pos = seResult.position + _model->getComPosWorld(); // world frame
          _com_pos = _model->_state.bodyPosition + _model->getComPosWorld(); // world frame
          _com_vel = _model->getComVel();


          des_location_<< _x_com_des, _y_com_des; //_com_pos[0], _com_pos[1];
          _com_pos_ini << _com_pos;
          _com_vel_ini << _com_vel;
          _stance_foot_loc_ini << stand_foot_pos;
          // std::cout << "_com_pos: " << _com_pos << '\n';
          // std::cout << "_com_vel: " << _com_vel  << '\n';
          // std::cout << "stand_foot_pos: " << stand_foot_pos  << '\n';

          pl_param.swing_time = swingTimeRemaining[i]; //0.33/2;
          pl_param.des_loc = des_location_;
          pl_param.stance_foot_loc = stand_foot_pos;
          // pl_param.b_positive_sidestep = swing_leg == 1;
          pl_param.b_positive_sidestep = swingStates[1]>0.0;
          // std::cout << "swing time: " << pl_param.swing_time<<'\n';

          // des_location_<< 0.0, 0.0;

          Vec3<float> target_loc;
          _planner->getNextFootLocation(_com_pos, // + stand_foot_pos,
                                        _com_vel,
                                        target_loc,
                                        &pl_param,
                                        &pl_output);


          target_loc[2] = 0;
          // static int ft = 0;
          // Vec3<float> fake_target;
          // fake_target << _com_pos;
          //
          // // fake_target[0] += 0.08*sin(2*M_PI*0.1*ft) -0.02;
          // fake_target[1] += -0.06*pow(-1, i);
          // // fake_target[1] += -pow(-1, i)*(0.07 + 0.05*cos(2*M_PI*0.1*ft));
          // fake_target[2] = 0.0;
          // ft++;

          for(int xy = 0; xy < 2; xy++){
            Pf[xy] = target_loc[xy];
            _tvr_lcm.foot_landing_loc_des[xy] = target_loc[xy];
            // Pf[xy] = fake_target[xy];
            // _tvr_lcm.foot_landing_loc_des[xy] = fake_target[xy];
          }
          // float t = 0.0;
          // for(int f(0); f<10; f++){
          //     _planner->getDesiredCOMState(
          //       t,
          //       _com_pos_ini,  _com_vel_ini,
          //       _stance_foot_loc_ini,
          //       _com_pos_state, _com_vel_state);
          //       for(int k(0); k<2; k++){
          //         _tvr_lcm.COM_pos_des[f][k] = _com_pos_state[k];
          //         _tvr_lcm.COM_vel_des[f][k] = _com_vel_state[k];
          //
          //       }
          //
          //   t +=  swingTimeRemaining[i]/10;
          // }
          for(int k(0); k<2; k++){
            _tvr_lcm.COM_pos_ini[k] = _com_pos[k];
            _tvr_lcm.COM_vel_ini[k] = _com_vel[k];
            _tvr_lcm.stance_foot_loc_ini[k] = stand_foot_pos[k];
          }
          // t  = fmod(t+dt, _swing_time);
          // if(_step_count>0){
          //
          //   _planner->getDesiredCOMState(
          //     t,
          //     _com_pos_ini,  _com_vel_ini,
          //     _stance_foot_loc_ini,
          //     _com_pos_state, _com_vel_state);
          //
          // }

          // Pf[2] = -0.002;
          Pf[2] = 0.0;
          liftSwingTrajectories[i].setMiddlePosition(pFoot[i]);
          liftSwingTrajectories[i].setFinalPosition(Pf);
        }else{

          _planner->getDesiredCOMState(
            0.5*_swing_time - swingTimeRemaining[i],
            _com_pos_ini,  _com_vel_ini,
            _stance_foot_loc_ini,
            _com_pos_state, _com_vel_state);
          // _com_pos = _model->_state.bodyPosition + _model->getComPosWorld(); // world frame
          // _com_vel = _model->getComVel();

            for(int k(0); k<2; k++){
              _tvr_lcm.COM_pos_des[k] = _com_pos_state[k];
              _tvr_lcm.COM_vel_des[k] = _com_vel_state[k];
            }

            // _tvr_lcm.COM_pos[2] = _com_pos[2];
            // _tvr_lcm.COM_vel[2] = _com_vel[2];
            // _tvr_lcm.step_count = _step_count;
            // _tvrLCM.publish("tvr_data_lcmt", &_tvr_lcm);
            // std::cout << "_com_pos: " << _com_pos_state  << '\n';
            // std::cout << "_com_pos: " << _com_vel_state  << '\n';
        }
      }
      else{
          //do nothing
          // printf("foot: %d t: %f\n", i, swingTimeRemaining[i]);
      }
    }
    else if(swingStates[(i+1)%2]>0.0){
      if(_step_count>0){

      _planner->getDesiredCOMState(
        1.5*_swing_time + iter_per_div*0.002 - swingTimeRemaining[(i+1)%2],
        _com_pos_ini,  _com_vel_ini,
        _stance_foot_loc_ini,
        _com_pos_state, _com_vel_state);

        for(int k(0); k<2; k++){

          _tvr_lcm.COM_pos_des[k] = _com_pos_state[k];
          _tvr_lcm.COM_vel_des[k] = _com_vel_state[k];

        }
        _tvr_lcm.COM_pos[2] = _com_pos[2];
        _tvr_lcm.COM_vel[2] = _com_vel[2];
        _tvr_lcm.step_count = _step_count;

        _tvrLCM.publish("tvr_data_lcmt", &_tvr_lcm);
        // std::cout << "_com_pos: " << _com_pos_state  << '\n';
        // std::cout << "_com_pos: " << _com_vel_state  << '\n';
      }

    }
    else{

    }

  }
  _tvr_lcm.step_count = _step_count;
  _tvrLCM.publish("tvr_data_lcmt", &_tvr_lcm);
  // gait
  Vec2<float> contactStates = gait->getContactState();
  Vec2<float> swingStates = gait->getSwingState();
  // Vec2<float> contactStates = sgait->getContactState();
  // Vec2<float> swingStates = sgait->getSwingState();
  Vec4<float> se_contactState; se_contactState.setZero();
  // std::cout << "iter: " <<_iter*0.002 << '\n';
  // pretty_print(contactStates, std::cout, "contactStates");
  // pretty_print(swingStates, std::cout, "swingStates");
  // printf("iter: %d t %f c: %f \n", _iter, 0.002*_iter, contactStates[0]);//, swingStates[0]);
  // if(_iter>3000)exit(0);
  // pretty_print(swingStates, std::cout, "swingStates: ");
  // pretty_print(contactStates, std::cout, "contactStates: ");
  for(size_t foot(0); foot < pat_biped::num_legs; ++foot)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];

    // if(swingState > 0) // foot is in swing
    // int swing_leg = swingStates[1]>0.0 && swingStates[1]<1.0? 1 : 0;
    // if(swing_leg == (int)foot) // foot is in swing
    // {
    if(swingStates[foot]>0.0) // foot is in swing
    {
      #ifdef DRAW_DEBUG_SWINGS

          auto* debugPath = data.visualizationData->addPath();
          if(debugPath) {
            debugPath->num_points = 100;
            debugPath->color = {1.0, 0.2, 0.2, 0.5};
            float step = (1.f - swingState) / 100.f;
            for(int i = 0; i < 100; i++) {
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
          CoMSphere->position = _com_pos;
          CoMSphere->position[2] = 0.0;
          CoMSphere->radius = 0.05;
          CoMSphere->color = {0.0, 0.0, 0.0, 0.5};
      #endif

      liftSwingTrajectories[foot].computeLiftSwingTrajectory(swingState, 0.5, swingTimes[foot]);
      Vec3<float> pDesFootWorld = liftSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = liftSwingTrajectories[foot].getVelocity();
      // if(foot == 0 && pDesFootWorld[2] < 0){
      //   pretty_print(swingStates, std::cout, "swingStates: ");
      //   std::cout << "pDesFootWorld: " << pDesFootWorld[2] << '\n';
      // }
      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = liftSwingTrajectories[foot].getAcceleration();
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
      se_contactState[foot] = contactState;
    }
  }
  // pBody_des << data._stateEstimator->getResult().position;
  pBody_des << _model->_state.bodyPosition;

  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  _updateWBCCMD();
  _iter++;
}

void WBC_BipedLocomotion::_updateWBCCMD(){

  pBody_des[2] = _body_height;

  vBody_des.setZero();
  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  contact_state = gait->getContactState();
  // contact_state = sgait->getContactState();
}

void WBC_BipedLocomotion::tvrParameterInitialization(const std::string & file){
  ParamHandler handler(file + ".yaml");

  handler.getValue("swing_time", _swing_time);
  handler.getValue("swing_height", _swing_height);
  handler.getValue("body_height", _body_height);
  handler.getValue("enable_double_stance", _enable_double_stance);
  handler.getVector("default_foot_location", _default_foot_loc);
  printf("[WBC_biped Planner] Parameter Setup is completed\n");
}
