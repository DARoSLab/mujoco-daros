#include <iostream>
#include <Timer.h>
#include <pretty_print.h>

#include "NonLinearMPCLocomotion.h"


//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH


////////////////////
// Controller
////////////////////

NonLinearMPCLocomotion::NonLinearMPCLocomotion(float _dt, int _iterations_between_mpc) :
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(20),
  dt(_dt),
  walking(20, Vec4<int>(0, 0, 0, 0), Vec4<int>(10, 10, 0, 0),"Walking"),
  standing(20, Vec4<int>(0,0,0,0),Vec4<int>(20, 20, 20, 20),"Standing"),
  nmpc()
{
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[NL MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);

  for(int i = 0; i < 4; i++) firstSwing[i] = true;

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();

  gait = &standing;
  
}

void NonLinearMPCLocomotion::run() {


  // get the state estimator result
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  

  if(firstRun)
  {
    // world_position_desired[0] = seResult.position[0];
    // world_position_desired[1] = seResult.position[1];
    computeInitialNMPCGuess();
    nmpc.initNMPC();
    nmpc.setInitialGuess(z0);
    firstRun = false;
  }


  if(_body_height < 0.02) {
    _body_height = _default_body_height;
  }

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  // v_des_world = seResult.rBody.transpose() * v_des_robot;
  // for(int i = 0; i < 4; i++) {
  //   pFoot[i] = seResult.position +
  //     seResult.rBody.transpose() * (data._summer->getHipLocation(i) +
  //         data._legController->datas[i]->p);
  // }

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // foot placement
  for(int l = 0; l < 4; l++) swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  float side_sign[4] = {-1, 1, -1, 1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  //float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    
    footSwingTrajectories[i].setHeight(0.06);
    


    // Vec3<float> pRobotFrame = (data._summer->getHipLocation(i) + offset);

    // pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    // float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    // Vec3<float> pYawCorrected =
    //   coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;


    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

  //   Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
  //       + des_vel * swingTimeRemaining[i]);

  //   // TODO - touched to try and increase the stance width a bit
  //   float p_rel_max = 0.5f;

  //   // Assume that v_des_robot is in the robot frame
  //   Vec2<float> pf_rel_body, pf_rel_world;
  //   Mat2<float> rotMatYaw;
  //   rotMatYaw << cos(seResult.rpy[2]), -sin(seResult.rpy[2]), sin(seResult.rpy[2]), cos(seResult.rpy[2]);
  //   double bonus_swing = _parameters->swing_bonus_x;
  //   if ((i == 2 || i == 3) && (seResult.vBody[0] >= 0.05) ) {
  //     bonus_swing = _parameters->hind_bonus_x;
  //   }
  //   // Raibert hueristic 
  //   pf_rel_body[0] = seResult.vBody[0] * (.5 + bonus_swing ) * stance_time + 
  //                   .03f*(seResult.vBody[0]-v_des_robot[0]) + 
  //                   (0.5f*seResult.position[2]/9.81f) * (seResult.vBody[1]*_yaw_turn_rate);
    
  //   pf_rel_body[1] = seResult.vBody[1] * (.5 + _parameters->swing_bonus_y) * stance_time + 
  //                    .03f*(seResult.vBody[1]-v_des_robot[1]) + 
  //                    (0.5f*seResult.position[2]/9.81f) * (-seResult.vBody[0]*_yaw_turn_rate);

  //   pf_rel_world = rotMatYaw * pf_rel_body;
  //   for(int xy = 0; xy < 2; xy++){
  //     pf_rel_world[xy] = fminf(fmaxf(pf_rel_world[xy], -p_rel_max), p_rel_max);
  //     Pf[xy] += pf_rel_world[xy];
  //   }
  //   Pf[2] = _parameters->swing_groundz;
  //   pNextFoot[i] = Pf;

  //   footSwingTrajectories[i].setFinalPosition(Pf);  
  //   footTrajGen[i].setFinalPosition(Pf);
  // }

  // gait
  contact_state = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();

  
  if (!mpc_can_run) {
    mpc_thread = std::thread(&NonLinearMPCLocomotion::updateMPCIfNeeded, this);
    mpc_can_run = true;
  }
  // updateMPCIfNeeded(mpcTable, data);
  iterationCounter++;


  for(int foot = 0; foot < 4; foot++)
  {
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
    }
  }
  // data._stateEstimator->setContactPhase(contact_state);

  // Update For WBC
  // std::cout<<"update WBC"<<std::endl;
  _updateWBC();
}

// void NonLinearMPCLocomotion::_updateWBC(){
//   // pBody_des[0] = world_position_desired[0];
//   // pBody_des[1] = world_position_desired[1];
//   pBody_des[0] = mpc_pBody_des[0];
//   pBody_des[1] = mpc_pBody_des[1];
//   pBody_des[2] = _body_height;

//   // vBody_des[0] = v_des_world[0];
//   // vBody_des[1] = v_des_world[1];
//   vBody_des[0] = mpc_vBody_des[0];
//   vBody_des[1] = mpc_vBody_des[1];
//   vBody_des[2] = 0.;

//   aBody_des.setZero();

//   pBody_RPY_des[0] = 0.;
//   pBody_RPY_des[1] = 0.;
//   pBody_RPY_des[2] = _yaw_des;

//   vBody_Ori_des[0] = 0.;
//   vBody_Ori_des[1] = 0.;
//   vBody_Ori_des[2] = _yaw_turn_rate;

// }

void NonLinearMPCLocomotion::updateMPCIfNeeded() {

  // const std::chrono::microseconds update_dt(int(_parameters->CAS_dt * 1000000));
  while (true) {
    // std::cout<<"mpc_can_run: "<<mpc_can_run<<std::endl;

    auto start = std::chrono::high_resolution_clock::now();
    int *mpcTable = gait->getMpcTable();
    auto seResult = _data._stateEstimator->getResult();

    float xStart = world_position_desired[0];
    float yStart = world_position_desired[1];
    MatrixXf mpcTable_full = Eigen::MatrixXf::Zero(4, _parameters->gait_segment);
    for(int i = 0; i < _parameters->gait_segment; i++)
    {
      for(int j = 0; j < 4; j++)
      {
        mpcTable_full(j,i) = mpcTable[i*4 + j];
      }
    }
    MatrixXf mpcTable_ = mpcTable_full.leftCols(horizonLength);
    // std::cout<<"[NLMPC] mpcTable_full: \n"<<mpcTable_full<<std::endl;

    MatrixXd contactLoc = Eigen::MatrixXd::Zero(12, horizonLength);

    Vec4<int> startSwing = Vec4<int>(0,0,0,0);
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < horizonLength; j++)
      {
        if (startSwing[i] == 0){
          contactLoc.block(3*i,j,3,1) = pFoot[i].cast<double>();
        } else {
          contactLoc.block(3*i,j,3,1) = pNextFoot[i].cast<double>();
        }
        if (mpcTable_(i,j) == 0){
          startSwing[i] = 1;
        }
        
      }
    }
    // std::cout<<"[NLMPC] contactLoc: \n"<<contactLoc<<std::endl;
    // std::cout<<"[NLMPC] contactLoc: \n"<<contactLoc<<std::endl;
    MatrixXf Rref   = get_desireR(seResult.rpy[2], _yaw_turn_rate);
    MatrixXf RPYref = get_desireRPY(seResult.rpy[2], _yaw_turn_rate);
    MatrixXf Xref   = get_desireX(Vector3f(xStart, yStart, _body_height), Vector3f(v_des_world[0], v_des_world[1], 0));
    VectorXf Vref(6);

    Vref << Vector3f(v_des_world[0], v_des_world[1], 0), seResult.rBody.transpose() * Vector3f(0, 0, _yaw_turn_rate);

    MatrixXf Fref = get_desireF(mpcTable_);
    // std::cout<<"[NLMPC] Fref: \n"<<Fref<<std::endl;

    Timer timer1;
    updateNMPCParams(_data, contactLoc.cast<float>(), mpcTable_ , Vref, Xref, Rref, Fref);
    nmpc.solve(params, 1);
    // std::cout<<"[NLMPC]total solve time: "<<timer1.getMs()<<std::endl;
    auto mpc_sol = nmpc.getSolution();
    auto z_sol = mpc_sol.z;
    auto init_cost = mpc_sol.init_cost;
    auto init_constraint_violation = mpc_sol.init_constraint_violation;
    auto final_cost = mpc_sol.final_cost;
    auto final_constraint_violation = mpc_sol.final_constraint_violation;
    auto linsearch_exitflag = mpc_sol.acceptance_type;
    auto step_length = mpc_sol.step_length;

    Eigen::Map<DMat<OSQPFloat>> x_sol(z_sol.data(), 18, horizonLength + 1);
    Eigen::Map<DMat<OSQPFloat>> fr_sol(z_sol.data() + 18 * (horizonLength + 1), 3 * 4, horizonLength);

    Fr_des[0] = fr_sol.col(0).head(3).cast<float>();
    Fr_des[1] = fr_sol.col(0).segment(3,3).cast<float>();
    Fr_des[2] = fr_sol.col(0).segment(6,3).cast<float>();
    Fr_des[3] = fr_sol.col(0).tail(3).cast<float>();

    mpc_pBody_des = x_sol.col(1).head(3).cast<float>();
    mpc_vBody_des = x_sol.col(1).segment(3,3).cast<float>();
    // std::cout<<"[NLMPC] Fr_des: "<<fr_sol.col(0).transpose()<<std::endl;

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    // std::cout << "MPC runtime: " << (duration.count()*0.000001) << " seconds" << std::endl;
    // std::this_thread::sleep_for(update_dt - duration);
    if (terminate_thread) {
      break;
    }
  }
}

void NonLinearMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
  iterationCounter = 0;
  terminate_thread = false;
}

void NonLinearMPCLocomotion::terminate(){
  mpc_can_run = false;
  terminate_thread = true;
  mpc_thread.join();
}



// MODE CHANGE HERE
// see how hright stuff works
void NonLinearMPCLocomotion::_setCommand(ControlFSMData<float> & data){
  _body_height = _default_body_height; // Nominal 

  float x_vel_cmd, y_vel_cmd;

  _yaw_turn_rate = -data._desiredStateCommand->data.stateDes(11);
  _x_vel_des = data._desiredStateCommand->data.stateDes(6);
  _y_vel_des = data._desiredStateCommand->data.stateDes(7);
  _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;

  // std::cout<<"_x_vel_des: "<<_x_vel_des<<std::endl;
  // std::cout<<"_y_vel_des: "<<_y_vel_des<<std::endl;
}


MatrixXf NonLinearMPCLocomotion::get_desireR(float yaw, float yaw_rate){
    Eigen::MatrixXf xR_des =  Eigen::MatrixXf::Zero(9, _parameters->CAS_pred_hor+1);

    for (int i = 0; i < _parameters->CAS_pred_hor+1; i++){
        float yawi = yaw + yaw_rate * i * _parameters->CAS_dt;
        Eigen::Matrix3f yaw_R;
        yaw_R << cos(yawi), -sin(yawi), 0,
                       sin(yawi), cos(yawi), 0,
                       0, 0, 1;

        Eigen::VectorXf xRk_des_vec = Eigen::Map<VectorXf>(yaw_R.data(), 9);
        xR_des.col(i) = xRk_des_vec;
    }

    return xR_des;
}

MatrixXf NonLinearMPCLocomotion::get_desireRPY(float yaw, float yaw_rate){
    Eigen::MatrixXf RPY_des =  Eigen::MatrixXf::Zero(3, _parameters->CAS_pred_hor+1);

    for (int i = 0; i < _parameters->CAS_pred_hor+1; i++){
        float yawi = yaw + yaw_rate * i * _parameters->CAS_dt;
        Eigen::Vector3f RPYk_des(0, 0, yawi);

        RPY_des.col(i) = RPYk_des;
    }
    
    return RPY_des;
}


MatrixXf NonLinearMPCLocomotion::get_desireX(Vector3f startPos, Vector3f desVel){
    MatrixXf x_des = startPos.replicate(1, _parameters->CAS_pred_hor+1);

    for (int i = 0; i < _parameters->CAS_pred_hor+1; i++){
        Vector3f dist =  i * _parameters->CAS_dt * desVel;
        x_des.col(i) = startPos + dist;
    }
    return x_des;
}

MatrixXf NonLinearMPCLocomotion::get_desireF(MatrixXf mpcTable){
    MatrixXf f_des = MatrixXf::Zero(12, _parameters->CAS_pred_hor);

    for (int i = 0; i < _parameters->CAS_pred_hor; i++){
        for (int j = 0; j < 4; j++){
            if (mpcTable(j, i) == 1) {
              f_des(3*j+2, i) = 11.791459 * 9.81 / mpcTable.col(i).sum();
            }
        }
    }
    return f_des;
}

void  NonLinearMPCLocomotion::computeInitialNMPCGuess(){
  DVec<OSQPFloat> x_init = DVec<OSQPFloat>::Zero(18);
  DVec<OSQPFloat> f_init = DVec<OSQPFloat>::Zero(12);
  // position, velocity, angular velocity, orientation
  x_init << 0, 0, _default_body_height, 0, 0, 0, 0, 0, 0,
            1, 0, 0, 0, 1, 0, 0, 0, 1;
  // reaction force
  f_init << 0, 0, 32.1, 0, 0, 32.1, 0, 0, 32.1, 0, 0, 32.1;

  DMat<OSQPFloat> x_mpc = DMat<OSQPFloat>::Zero(18, horizonLength + 1);
  DMat<OSQPFloat> f_mpc = DMat<OSQPFloat>::Zero(12, horizonLength);
  for (int i = 0; i < horizonLength + 1; i++){
    x_mpc.col(i) = x_init;
  }
  for (int i = 0; i < horizonLength; i++){
    f_mpc.col(i) = f_init;
  }
  DVec<OSQPFloat> z(x_mpc.size() + f_mpc.size());
  z << Eigen::Map<DVec<OSQPFloat>>(x_mpc.data(), x_mpc.size()),
        Eigen::Map<DVec<OSQPFloat>>(f_mpc.data(), f_mpc.size());
  z0.resize(z.size());
  std::copy(z.data(), z.data() + z.size(), z0.begin());

}

void NonLinearMPCLocomotion::updateNMPCParams(ControlFSMData<float> &data, MatrixXf contactLoc, MatrixXf mpcTable, VectorXf Vref, MatrixXf Xref, MatrixXf Rref, MatrixXf Fref){
  // update NMPC parameters
  auto seResult = data._stateEstimator->getResult();
  auto pos = seResult.position.template cast<OSQPFloat>();
  auto vel = seResult.vWorld.template cast<OSQPFloat>();
  auto omega = seResult.omegaBody.template cast<OSQPFloat>();
  Mat3<float> w_R_b = seResult.rBody.transpose();
  Eigen::Map<Eigen::Matrix<float, 9, 1>> R9by1(w_R_b.data());
  auto r = R9by1.template cast<OSQPFloat>();
  DVec<OSQPFloat> init_state = DVec<OSQPFloat>::Zero(18);
  init_state << pos, vel, omega, r;
  // convert contactLoc from matrix to vector
  Eigen::VectorXf contactLocvec = Eigen::Map<Eigen::VectorXf>(contactLoc.data(), contactLoc.size());
  DVec<OSQPFloat> contactLocvecOSQP = contactLocvec.template cast<OSQPFloat>();
  // convert mpcTable from matrix to vector
  Eigen::VectorXf mpcTablevec = Eigen::Map<Eigen::VectorXf>(mpcTable.data(), mpcTable.size());

  DVec<OSQPFloat> mpcTablevecOSQP = mpcTablevec.template cast<OSQPFloat>();
  // convert Xref from matrix to vector
  Eigen::VectorXf Xrefvec = Eigen::Map<Eigen::VectorXf>(Xref.data(), Xref.size());
  DVec<OSQPFloat> XrefOSQP = Xrefvec.template cast<OSQPFloat>();
  // convert Rref from matrix to vector
  Eigen::VectorXf Rrefvec = Eigen::Map<Eigen::VectorXf>(Rref.data(), Rref.size());
  DVec<OSQPFloat> RrefOSQP = Rrefvec.template cast<OSQPFloat>();
  // convert Vref from vector to osqp vector
  DVec<OSQPFloat> VrefOSQP = Vref.template cast<OSQPFloat>();
  // convert Fref from matrix to vector
  Eigen::VectorXf Frefvec = Eigen::Map<Eigen::VectorXf>(Fref.data(), Fref.size());
  DVec<OSQPFloat> FrefOSQP = Frefvec.template cast<OSQPFloat>();

  params.clear();
  params.insert(params.end(), init_state.data(), init_state.data() + init_state.size());
  params.insert(params.end(), contactLocvecOSQP.data(), contactLocvecOSQP.data() + contactLocvecOSQP.size());
  params.insert(params.end(), mpcTablevecOSQP.data(), mpcTablevecOSQP.data() + mpcTablevecOSQP.size());
  params.insert(params.end(), XrefOSQP.data(), XrefOSQP.data() + XrefOSQP.size());
  params.insert(params.end(), RrefOSQP.data(), RrefOSQP.data() + RrefOSQP.size());
  params.insert(params.end(), FrefOSQP.data(), FrefOSQP.data() + FrefOSQP.size());
  params.insert(params.end(), VrefOSQP.data(), VrefOSQP.data() + VrefOSQP.size());
  VectorXf weight = VectorXf::Ones(13);
  weight << _parameters->CAS_Q_X.cast<float>(), _parameters->CAS_Q_Xd.cast<float>(), _parameters->CAS_Q_R.cast<float>(), _parameters->CAS_Q_W.cast<float>(), float(_parameters->CAS_Q_U);
  DVec<OSQPFloat> weightOSQP = weight.template cast<OSQPFloat>();
  params.insert(params.end(), weightOSQP.data(), weightOSQP.data() + weightOSQP.size());
  VectorXf inertia = VectorXf::Ones(4);
  inertia << 0.116940, 0.352276, 0.376479, 11.791459;
  DVec<OSQPFloat> inertiaOSQP = inertia.template cast<OSQPFloat>();
  params.insert(params.end(), inertiaOSQP.data(), inertiaOSQP.data() + inertiaOSQP.size());
}
