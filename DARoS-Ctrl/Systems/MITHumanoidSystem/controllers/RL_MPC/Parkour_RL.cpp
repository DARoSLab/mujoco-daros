#include "Parkour_RL.h"
#include <iostream>
#include <Utilities/Timer.h>
#include <pretty_print.h>
#include "MPC_Casadi/eigenHelper.hpp"
#include <cppTypes.h>
#include <loco-jump/utils/State2d.hpp>
template <typename T>
Parkour_RL<T>::Parkour_RL(
    float _dt, int _iterations_between_mpc, TelloParameters* parameters,
     FloatingBaseModel<T> * model, ControlFSMData_Tello<T>* _fsm_data) :
  _model(model),
  _fsm_data(_fsm_data),
  iterationsBetweenMPC(_iterations_between_mpc),
  pred_hor(parameters->CAS_pred_hor),
  dt(_dt)
{
  p = parameters;

  dtMPC = dt * iterationsBetweenMPC;

  for(int i = 0; i < tello_contact::num_foot_contact; i++)
  {
    firstSwing[i] = true;
    pFoot_des[i].setZero();
    vFoot_des[i].setZero();
    aFoot_des[i].setZero();
  }

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
  rough_terrain_start =  floor(1 / terrain_density);
  rough_terrain_end = floor(30 / terrain_density);
  generate_terrain();
  animator.send_terrain(terrain);
  gaitManager = new gaitManagerRL(p, _iterations_between_mpc, terrain);


}
template <typename T>
void Parkour_RL<T>::init_casadi(){
  optRL.init(_model, p);
}

template <typename T>
void Parkour_RL<T>::run(ControlFSMData_Tello<T>& data) 
{

  auto& seResult = data._stateEstimator->getResult();
  Vec3<float> bodyframePose = seResult.position;
  Mat3<float> b_R_w = seResult.rBody;
  Mat3<float> w_R_b = b_R_w.transpose();
  Vec3<float> comPos = bodyframePose + w_R_b* _model->getComPos();//COM in the world frame
  // std::cout<<"vel: \n"<< seResult.vWorld << std::endl;
  MPC_State = gaitManager->getStates();
  if (MPC_State == 2) {
    // try to get the state for the RL
    // x position and z position 
    s2d.x << double(comPos[0]), double(comPos[2]);
    // x vel and z vel
    s2d.xd << double(seResult.vWorld[0]), double(seResult.vWorld[2]);
    // std::cout<<"xd: "<< s2d.xd.transpose() << std::endl;
    // pitch angle and velocity around y axis
    s2d.theta = seResult.rpy[1]; s2d.w = seResult.omegaWorld[1];
    // getting the position of the current right foot, it shall be on the ground
    Vec3<float> R_heel_pose = _model->_pGC[tello_contact::R_heel];
    Vec3<float> R_toe_pose = _model->_pGC[tello_contact::R_toe];
    Vec3<float> R_foot_pose = (R_heel_pose * 0.055 + R_toe_pose * 0.05) / 0.105;

    s2d.curr_contact_loc = Vec2<double>(R_foot_pose[0], R_foot_pose[2]);
    // std::cout<<"curr_contact_loc: \n"<< s2d.curr_contact_loc << std::endl;

    s2d.terrain_obs = get_terrain_obs(s2d.curr_contact_loc[0]);
    int idx = (int)floor(s2d.curr_contact_loc[0] / terrain_density); 
    if (idx >= rough_terrain_end){
      s2d.rough_terrain_ends = 1;
    } else {
      s2d.rough_terrain_ends = 0;
    }

    // std::cout<<"terrain_obs: \n"<< s2d.terrain_obs << std::endl;
    VectorXd act2d = agent.module_forward(s2d);
    desired_vel = act2d(0);
    // std::cout<<"desired_vel: "<< desired_vel << std::endl;
    // std::cout<<"act2d: \n"<< act2d << std::endl;
    ContactData cd = gaitManager->rl2dActToContactData(act2d, float(s2d.curr_contact_loc[0]));
    gaitManager->contactTable = cd.contactTable;
    gaitManager->xlocationTable = cd.xlocationTable;
    gaitManager->zlocationTable = cd.zlocationTable;
    gaitManager->contactTableNext = Eigen::MatrixXd::Ones(4, 1) * -1;
    gaitManager->xlocationTableNext = Eigen::MatrixXd::Ones(4, 1) * -1;
    gaitManager->zlocationTableNext = Eigen::MatrixXd::Ones(4, 1) * 0;

  }
  // std::cout<<"desired_vel: "<< desired_vel << std::endl;
  gaitManager->updates();
  // std::cout<<"right heel position; "<< _model->_pGC[tello_contact::R_heel].transpose() << std::endl;
  // std::cout<<"right toe position; "<< _model->_pGC[tello_contact::R_toe].transpose() << std::endl;
  // std::cout<<"left heel position; "<< _model->_pGC[tello_contact::L_heel].transpose() << std::endl;
  // std::cout<<"left toe position; "<< _model->_pGC[tello_contact::L_toe].transpose() << std::endl;
  // std::cout<<"swingstate:";
  for (int i=0; i<4; i++) {
    pCurContact[i] =  _model->_pGC[i+tello_contact::R_heel];
    contact_state[i] = gaitManager->isSwing[i];
    // std::cout<<gaitManager->isSwing[i]<<",";
  }
  // std::cout<<gaitManager->mpcTable<<std::endl;
  // std::cout<<"right heel position; "<< _model->_pGC[tello_contact::R_heel].transpose() << std::endl;
  // std::cout<<std::endl;

  for (int i = 0; i <tello_contact::num_foot_contact; i++)
  {
    if (gaitManager->isSwing[i] == false){
      pFoot_des[i] = pCurContact[i];
      vFoot_des[i] = Eigen::Vector3f::Zero(3);
      aFoot_des[i] = Eigen::Vector3f::Zero(3);      
      firstSwing[i] = true;
    } 
    else 
    {
      if (firstSwing[i]){
        firstSwing[i] = false;
        gaitManager->setInitialPosition(pCurContact[i], i);
        // std::cout<<"initial position: \n"<< pCurContact[i] << std::endl;
      }
      if (gaitManager->zlandingloc[i] == 0){
        gaitManager->setHeight(p->gait_swing_height, i);
      } else {
        gaitManager->setHeight(p->stairstep_height, i);
      }
      
      Vec3<float> contactOffset (
        (i ==0 || i == 2 ) ? p->gait_heel_offset: p->gait_toe_offset,
        (i <=1) ? -p->gait_foot_offset : p->gait_foot_offset,
        0.0
      );

      Vec3<float> footLoc(gaitManager->xlandingloc[i], comPos[1] + seResult.vWorld[1] *0.35, gaitManager->zlandingloc[i]);
      Vec3<float> contactLoc = footLoc + contactOffset;
      // if (i == 0){
      //   std::cout<<"desired location: "<< contactLoc.transpose() << std::endl;
      // }
      gaitManager->setFinalPosition(contactLoc, i);
      float phase = gaitManager->swingPhase[i];

      // if (gaitManager->zlandingloc[i] > gaitManager->prezlandingloc[i]){
      //   if (phase > 0.5 && phase < 1.0) {
      //     phase = 1 - (1.0 - phase) * 0.5;
      //   }
      // }
      // std::cout<< i << " final : "<< contactLoc.transpose() <<  "cur pose: " << pCurContact[i].transpose()<< "phase: "<< phase<<"swing time "<< gaitManager->swingTimes[i]<< std::endl;

      gaitManager->computeSwingTrajectoryBezier(phase, gaitManager->swingTimes[i], i);
      pFoot_des[i] = gaitManager->_p[i];
      vFoot_des[i] = gaitManager->_v[i];
      aFoot_des[i] = gaitManager->_a[i];
      // std::cout<<"vFoot_des: \n"<< vFoot_des[i] << std::endl;
      // std::cout<<"aFoot_des: \n"<< aFoot_des[i] << std::endl;

    }
  }

    if (_iter % iterationsBetweenMPC == 0) 
    {
      pred_hor = gaitManager->mpcTable.cols();
      Eigen::MatrixXd contactTable = Eigen::MatrixXd::Zero(3 * tello_contact::num_foot_contact, pred_hor);
      for (int i = 0; i <tello_contact::num_foot_contact; i++) 
      {
        bool useCurCon = true; // by default, if initially the foot is undercontact, we shall just use the current location
        for (int j = 0; j < pred_hor; j++)
        {
          if (gaitManager->mpcTable(i,j) == 1 && useCurCon)
          {
            contactTable.block(3*i, j, 3, 1) = pCurContact[i].cast<double>();
          } 
          else if (gaitManager->mpcTable(i,j) == 1 && useCurCon == false) 
          {
            Vec3<float> contactOffset 
            (
              (i ==0 || i == 2 ) ? p->gait_heel_offset : p->gait_toe_offset,
              (i <=1) ? -p->gait_foot_offset : p->gait_foot_offset,
              0.0
            );
            Vec3<float> footLoc(gaitManager->mpcLocTableX(i,j), comPos[1], gaitManager->mpcLocTableZ(i,j));
            Vec3<float> contactLoc = footLoc + contactOffset;
            contactTable.block(3*i, j, 3, 1) = contactLoc.cast<double>();
          } else {
            useCurCon = false;
          }
        }
      }
      // std::cout<<"current position: \n"<< comPos << std::endl;

      // std::cout<<"contactTable: \n"<< contactTable << std::endl;
      
      State_MPC s_mps;
      s_mps.x = EigenVectorfTodm(comPos);
      s_mps.xd = EigenVectorfTodm(seResult.vWorld);
      Eigen::Map<Eigen::Matrix<float, 9, 1>> R9by1(w_R_b.data());
      s_mps.R = EigenVectorfTodm(R9by1);
      s_mps.w = EigenVectorfTodm(seResult.omegaBody);
      s_mps.contactLoc = EigenMatrixTodm(contactTable);
      s_mps.mpcTable = EigenMatrixTodm(gaitManager->mpcTable);

      VectorXf ref(15);
      Vector3f cas_x_des = p->CAS_x_des;
      Vector3f cas_v_des = p->CAS_v_des; cas_v_des(0) = desired_vel;
      VectorXf cas_w_des = p->CAS_w_des;
      Vector3f CAS_RPY_des = p->CAS_RPY_des;    
      Eigen::Matrix3f cas_R_des = ori::rpyToRotMat(CAS_RPY_des).transpose();
      Eigen::Map<Eigen::Matrix<float, 9, 1>> cas_R_des_vec(cas_R_des.data());
      ref <<  cas_v_des, cas_w_des, cas_R_des_vec;
      MatrixXf xref = get_desireX(desired_vel, comPos[0]);
      // std::cout<<"desired_vel: \n"<< desired_vel << std::endl;
      // std::cout <<"contactTable: \n"<< contactTable << std::endl;
      // std::cout <<"mpcTable: \n"<< gaitManager->mpcTable << std::endl;
      // std::cout<<"xref: \n"<< xref << std::endl;
      MPC_output output = optRL.optimize(_model, p, s_mps, ref, xref);

      
      //order matters
      Fr_des[0] = output.F_RHeel.col(0);
      Fr_des[1] = output.F_RToe.col(0);
      Fr_des[2] = output.F_LHeel.col(0);
      Fr_des[3] = output.F_LToe.col(0);
      // std::cout<<"x: \n"<< output.x << std::endl;
      // std::cout<<"XD: \n"<< output.xd << std::endl;
      // std::cout<<"current velocity: \n"<< seResult.vWorld << std::endl;
      pBody_des     = output.x.col(1);
      vBody_des     = output.xd.col(1);
      aBody_des     = Eigen::Vector3f::Zero(3);
      pBody_RPY_des = output.RPY.col(1);
      vBody_Ori_des = output.xw.col(1);

      // if we will run out of the state, then we need to get rl action based on the mpc's prediction. 
      if (MPC_State == 1){
        // get the index of prediciton from the mpc
        int pred_idx = gaitManager->contactTable.cols() - gaitManager->idxTable;
        State2d pred_s2d; 
        pred_s2d.x << output.x(0, pred_idx), output.x(2, pred_idx);
        pred_s2d.xd << output.xd(0, pred_idx), output.xd(2, pred_idx);

        //get 3by3 matrix from out.xR.col(pred_idx)
        Eigen::Matrix3d pred_R  = (Map<Matrix3f>(output.xR.col(pred_idx).data())).cast<double>();


        pred_s2d.theta = output.RPY(1, pred_idx); 
        Eigen::Vector3d omega_body = (output.xw.col(pred_idx)).cast<double>();
        Eigen::Vector3d omega_world = pred_R * omega_body;
        pred_s2d.w = omega_world[1];
        // std::cout<<"omega_body: \n"<< omega_body << std::endl;
        // std::cout<<"omega_world: \n"<< omega_world << std::endl;
        // wheter use contact from the RL or use my acutal contact should be implmeneted in contactTable
        float R_heel_x = contactTable(0, pred_idx-1);
        float R_toe_x = contactTable(3, pred_idx-1);
        double R_foot_x = (double)(R_heel_x * 0.055 + R_toe_x * 0.05) / 0.105;
        double R_foot_z = get_contact_y(R_foot_x);
        pred_s2d.curr_contact_loc = Vec2<double>(R_foot_x, R_foot_z);

        pred_s2d.terrain_obs = get_terrain_obs(pred_s2d.curr_contact_loc(0));
        int idx = (int)floor(pred_s2d.curr_contact_loc[0] / terrain_density); 
        if (idx >= rough_terrain_end){
          pred_s2d.rough_terrain_ends = 1;
        } else {
          pred_s2d.rough_terrain_ends = 0;
        }
        VectorXd act2d = agent.module_forward(pred_s2d);
        // std::cout<<"act2d based on pred: \n"<< act2d << std::endl;
        ContactData cd = gaitManager->rl2dActToContactData(act2d, float(pred_s2d.curr_contact_loc[0]));
        gaitManager->contactTableNext = cd.contactTable;
        gaitManager->xlocationTableNext = cd.xlocationTable;
        gaitManager->zlocationTableNext = cd.zlocationTable;

      }

    }
  _iter ++;
  gaitManager->setIterations(_iter);
}

template<typename T>
void Parkour_RL<T>::generate_terrain(){
      // first of all set everything to be 1, which means flat ground

    int num_tile = 2000;
    terrain = Eigen::VectorXd::Zero(num_tile);
    // then randomly generate some pits, 
    srand (time(NULL));
    int pit = 22;
    add_stair(pit+4);
    add_pit(pit+74, 6);
    add_pit(pit+104, 6);
    add_stair(pit+134);

    // for (int i = rough_terrain_start;  i<rough_terrain_end; i+=60){

    //     int pit = 4 + rand() % 6 + i;
    //     int choice = rand() % 2;

    //     switch (choice){
    //         // changing everyhing to 2 instead of 2, 3, 5
    //         case 0:
    //             add_pit(pit+5, 6);
    //             add_pit(pit+30, 6);
    //             break;
    //         case 1:
    //             add_stair(pit);
    //             break;
    //     }
    // }
    std::cout<< terrain.transpose() << std::endl;

}
template<typename T>
void Parkour_RL<T>::add_pit(int start, int len){
    for (int i=start; i<start+len; i++){
        terrain(i) = -1;
    }
}

template<typename T>
void Parkour_RL<T>::add_stair(int start){
    for (int i=start+1; i<start+8; i++){
        terrain(i) = 0.08;//.1*0.8;
    }
    for (int i=start+8; i<start+15; i++){
        terrain(i) = 0.16;//.2*0.8;
    }
    for (int i=start+15; i<start+28; i++){
        terrain(i) = 0.24;//.3*0.8;
    }
    for (int i=start+28; i<start+35; i++){
        terrain(i) = 0.16;//.2*0.8;
    }
    for (int i=start+35; i<start+42; i++){
        terrain(i) = 0.08;//.1*0.8;
    }
}

template<typename T>
VectorXd Parkour_RL<T>::get_terrain_obs(double x_loc){
    
    double x = (x_loc<0)?0:x_loc;
    int idx = (int)floor(x / 0.05);
    VectorXd obs(20);
    for (int i=1; i<20+1; i++){
        if (terrain[idx+i] < 0) {
            obs(i-1) = terrain[idx+i];
        } else {
            obs(i-1) = terrain[idx+i] - terrain[idx];
        }
    }
    return obs;
}

template<typename T>
MatrixXf Parkour_RL<T>::get_desireX(double desired_vel, float x){
    MatrixXf xk_des = p->CAS_x_des.replicate(1, pred_hor+1);

    for (int i = 0; i < pred_hor+1; i++){
        double dist = i * p->CAS_dt * desired_vel;
        double height = get_contact_y(x+  dist);
        // height = (height < -0.5)? 0: height;
        if (height < 0){
            double pre_height = get_contact_y(x +  dist - 0.05);
            double post_height = get_contact_y(x +  dist + 0.05);
            if (pre_height >0 || post_height >0){
                height = 0.5 * (pre_height + post_height);
            } else {
                height = 0;
            }
        }
        xk_des(0, i) = x + dist;

        xk_des(2, i) =  xk_des(2, i) + height ;
    }

    return xk_des;
}

template<typename T>
double Parkour_RL<T>::get_contact_y(double x_loc){
    double x = (x_loc<0)?0:x_loc;
    int idx = (int)floor(x / 0.05);
    return terrain[idx];
}

template class Parkour_RL<float>;

