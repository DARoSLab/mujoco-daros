/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */
#ifdef RMPFLOW_BUILD
#include "FSM_State_RMP.h"
#include <wbc_ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <convexMPC_Biped/LiftSwingTrajectory.hpp>
#include <common/control/RMPFlow/LeafNodeSet/GoalAttractorUniRMP3D.hpp>
#include <common/control/RMPFlow/LeafNodeSet/RedundancyRMP.hpp>
#include <common/control/RMPFlow/LeafNodeSet/ForwardKinematicsRMP.hpp>
#include <common/control/RMPFlow/LeafNodeSet/CollisionAvoidanceRMP3D.hpp>
#include <common/control/RMPFlow/LeafNodeSet/LinkAttractorRMP.hpp>
#include <common/control/RMPFlow/LeafNodeSet/LinkCollisionRMP.hpp>
#include <common/control/RMPFlow/LeafNodeSet/LinkFKRMP.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_RMP<T>::FSM_State_RMP(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND,"BALANCE_STAND") {
  // Set the pre controls safety checks
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF to 0s
  this->footFeedForwardForces = Mat32<T>::Zero();
  _model = this->_data->_pat->buildModel();
  _state.q = DVec<T>::Zero(_model._nDof-6);
  _state.qd = DVec<T>::Zero(_model._nDof-6);
  _qddot = DVec<T>::Zero(_model._nDof);
  _tau_ff = DVec<T>::Zero(_model._nDof-6);
  _q_des = DVec<T>::Zero(_model._nDof-6);
  _qdot_des = DVec<T>::Zero(_model._nDof-6);
  Vec3<T> zero_3; zero_3.setZero();

  // _shank_offsets[0][0]<<0.0, 0, 0;
  _shank_offsets[0][0]<<0.0, 0.0, 0.0;
  _shank_offsets[0][1]<<0.0, -0.004, -0.1;
  _shank_offsets[0][2]<<0.0, -0.004, -0.15;
  _shank_offsets[0][3]<<0.0, -0.004, -0.20;
  // _shank_offsets[1][0]<<0.0, 0, 0;
  _shank_offsets[1][0]<<0.0, 0.0, 0.0;
  _shank_offsets[1][1]<<0.0, 0.004, -0.1;
  _shank_offsets[1][2]<<0.0, 0.004, -0.15;
  _shank_offsets[1][3]<<0.0, 0.004, -0.20;



  // _rootRMP = new RMPRoot<float>("Root", _model._nDof-6);
  // _rf_FKRMP = new ForwardKinematicsRMP<float>("LF_FK", 3, _rootRMP, &_model, pat_biped_linkID::RF, zero_3);
  // _lf_FKRMP = new ForwardKinematicsRMP<float>("RF_FK", 3, _rootRMP, &_model, pat_biped_linkID::LF, zero_3);
  // _lf_attractorRMP = new GoalAttractorUniRMP3D<float>("LF_Attractor", 3, _lf_FKRMP, pat_biped_linkID::LF);
  // _rf_attractorRMP = new GoalAttractorUniRMP3D<float>("RF_Attractor", 3, _rf_FKRMP, pat_biped_linkID::RF);
  // _lf_collisionRMP_1 = new CollisionAvoidanceRMP3D<float>("LF_Collision", 1, _lf_FKRMP, pat_biped_linkID::LF);
  // _lf_redundancyRMP = new RedundancyRMP<float>("LF_redundancy", 3, _rootRMP, &_model, pat_biped_linkID::LF);
  // _rf_redundancyRMP = new RedundancyRMP<float>("RF_redundancy", 3, _rootRMP, &_model, pat_biped_linkID::RF);

  // DVec<float> f_goal; f_goal.resize(3);
  // f_goal << -0.0, 0.2, -0.45;
  // _rf_attractorRMP->update_parameters(f_goal);
  // f_goal << -0.0, -0.2, -0.45;
  // _lf_attractorRMP->update_parameters(f_goal);
  // _collision_1.resize(3);
  // _collision_1 << 0.0, 0.03, -0.35;
  // _lf_collisionRMP_1->update_parameters(_collision_1);
  // for(int c(0); c<3; ++c){
  //   _lf_c[c] = new CollisionAvoidanceRMP3D<float>("lfc", 1, _lf_FKRMP, 5);
  // }



}

template <typename T>
void FSM_State_RMP<T>::onEnter() {
  // Always set the gait to be standing in this state
  std::cout << "[RMP STEPPING] OnEnter" << '\n';
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;
  Vec4<T> contactState;
  contactState<< 0.5, 0.5, 0.5, 0.5;
  this->_data->_stateEstimator->setContactPhase(contactState);


  for(int leg(0); leg<2; ++leg){

    for (int i(0); i < 3; ++i){
      _state.q[3*leg + i] = this->_data->_legController->datas[leg]->q[i];
      _q_des(3*leg + i) = _state.q[3*leg + i];
      _state.qd[3*leg + i] = this->_data->_legController->datas[leg]->qd[i];
    }

  }
  _model.setState(_state);
  _model.forwardKinematics();
  _model.contactJacobians();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_RMP<T>::run() {
  // DVec<T> x, xdot;
  // x.resize(_model._nDof-6);
  // xdot.resize(_model._nDof-6);
  // x.resize(_rootRMP->getDim());
  // xdot.resize(_rootRMP->getDim());
  _state.bodyPosition = this->_data->_stateEstimator->getResult().position;
  for(int leg(0); leg<2; ++leg){

    for (int i(0);i < 3;++i){
      _state.q[3*leg + i] = this->_data->_legController->datas[leg]->q[i];
      _state.qd[3*leg + i] = this->_data->_legController->datas[leg]->qd[i];

        // x(_rootRMP->getDim() - 6 + 3*leg+i) = _state.q[3*leg + i];
        // xdot(_rootRMP->getDim() - 6 + 3*leg+i) = _state.qd[3*leg + i];

    }

  }
  _model.setState(_state);
  _model.forwardKinematics();
  _model.contactJacobians();
  _model.massMatrix();//Comment if using massandCoriolisMatrix()
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();
  _model.centroidMomentumMatrix();
  //_model.massandCoriolisMatrix();//Comment if not needed

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();

  BalanceStandStep();

  // DVec<T> q_lf_des; q_lf_des.resize(3);
  // DVec<T> q_rf_des; q_rf_des.resize(3);
  // q_lf_des<< 0.0, 0.0, 1.5*sin(2*M_PI*_iter*0.001);
  // q_rf_des<< 0.0, 0.0, 1.5*sin(2*M_PI*_iter*0.001);
  // _lf_redundancyRMP->update_parameters(q_lf_des);
  // _rf_redundancyRMP->update_parameters(q_rf_des);

  // _collision_1 <<_model._pGC[pat_biped_linkID::RF];
  // _lf_collisionRMP_1->update_parameters(_collision_1);
  //
  // DVec<float> f_goal; f_goal.resize(3);
  // f_goal << 0.0, -0.06, SwingTrajectory(_phase[0], 0.05)-0.40;
  // _rf_attractorRMP->update_parameters(f_goal);
  // f_goal << 0.0, 0.06, SwingTrajectory(_phase[1], 0.05)-0.40;
  // _lf_attractorRMP->update_parameters(f_goal);


  for(int leg(0); leg<2; ++leg){

    for(int c(0); c<4; ++c){

      int  clink_id = leg == 0? pat_biped_linkID::LF - 2: pat_biped_linkID::RF - 2; //leg 0 right leg

      _collision_loc[leg][c] << _model.getPosition(clink_id, _shank_offsets[leg][c]);
      // _lf_c[c]->update_parameters(_collision_loc[c]);
      // auto* collisionSphere = this->_data->visualizationData->addSphere();
      // collisionSphere ->position << _collision_loc[leg][c];
      // collisionSphere ->radius = 0.05;
      // collisionSphere ->color = {0.8, 0.0, 0.0, 0.5};

    }
  }

  _rootRMP = new RMPRoot<float>("Root", _model._nDof, &_model);
  Vec3<T>p1, p2, p3, p4;
  p1 << _model.getPosition(pat_biped_linkID::RS, _shank_offsets[0][0]);
  p2 << _model.getPosition(pat_biped_linkID::RS, _shank_offsets[0][3]);
  p3 << _model.getPosition(pat_biped_linkID::LS, _shank_offsets[1][0]);
  p4 << _model.getPosition(pat_biped_linkID::LS, _shank_offsets[1][3]);
  for(size_t leg(0); leg<2; ++leg){
    // if(_phase[leg] > M_PI){ // Contact
    //
    // }else{ // No Contact (swing)
      // _foot_FKRMP[leg] = new ForwardKinematicsRMP<float>("FRFK", 3, _rootRMP, &_model, 10 + leg*3, _zero_3);
      int link_id = leg == 0 ? pat_biped_linkID::RS : pat_biped_linkID::LS;
      Vec3<T> _foot_offset; _foot_offset << 0.0, 0.0, -0.2;
      _foot_FKRMP[leg] = new LinkFKRMP<float>("FootFK_" + std::to_string(leg), 3, _rootRMP, &_model, link_id, _foot_offset);
      _foot_attractorRMP[leg] = new LinkAttractorRMP<float>("FootAttractor_" + std::to_string(leg), 3, _foot_FKRMP[leg], 10 + leg*3);
      Vec3<T> wp1;
      Vec3<T> wp2;

      DVec<T> des_pos_vel_acc = DVec<T>::Zero(9);
      if(leg == 0)
        des_pos_vel_acc.head(3) << 0.04, -0.06  + SwingTrajectory(_phase[0], 0.15), 0.20; //SwingTrajectory(_phase[0], 0.05);
      else
        des_pos_vel_acc.head(3) << 0.04, -0.06 + SwingTrajectory(_phase[1], 0.15), 0.20; //SwingTrajectory(_phase[0], 0.05);

      _foot_attractorRMP[leg]->update_parameters(des_pos_vel_acc);

      auto* goalSphere = this->_data->visualizationData->addSphere();
      goalSphere->position << des_pos_vel_acc.head(3);
      goalSphere->radius = 0.05;
      goalSphere->color = {0.0, 0.8, 0.0, 0.5};

      // if(_phase[leg] <= M_PI){ //swing
        // if(leg == 1){
        if(_iter>5000){
          // for(int i(0); i<4; i++){
          //
          //   for(int c(0); c<4; c++){
          //     _left_shank_c[i][c] = new LinkCollisionRMP<float>("lf_c_" + std::to_string(c), 1, _foot_FKRMP[leg], 5);
          //     _left_shank_c[i][c]->update_parameters(_collision_loc[leg][c]);
          //   }
          //
          // }
          bool wps = find_shank_capsule_witness_pts(wp1, wp2);
          if(wps){

            Vec3<T> shank_offset = leg ==0 ? wp1-p1:wp2-p3;
            Vec3<T> collision_loc = leg ==0 ? wp2:wp1;

            Mat4<T> W_T_L;
            W_T_L.block(0, 0, 3, 3) <<_model.getOrientation(link_id); // Link orientation in world
            W_T_L.block(0, 3, 3, 1) <<_model.getPosition(link_id);
            W_T_L(3, 3) = 1.0;
            Vec4<T> shank_offset_h;
            shank_offset_h << shank_offset, 1.0;
            shank_offset = (W_T_L.transpose()*shank_offset_h).block(0,0, 3, 1);//offset in link frame

            _shank_FKRMP = new LinkFKRMP<float>("shankFK_" + std::to_string(leg), 3, _rootRMP, &_model, link_id, shank_offset);
            _shank_collisionRMP = new LinkCollisionRMP<float>("shankCollision" + std::to_string(0), 1, _shank_FKRMP, 5);
            _shank_collisionRMP->update_parameters(collision_loc);
          }
        }
        //
        // auto* collisionSphere = this->_data->visualizationData->addSphere();
        // collisionSphere ->position << _collision_loc[leg][4];
        // collisionSphere ->radius = 0.1;
        // collisionSphere ->color = {0.0, 0.0, 0.8, 0.5};
      // }

      // }
    // }
  }

  // _rootRMP->solve(x, xdot);
  _rootRMP->solve();
  // if(_iter>2000){
    // std::cout << "_shank_FKRMP f: " << _shank_FKRMP->getF()<< '\n';
    // std::cout << "_shank_collisionRMP M: " << _shank_collisionRMP->getM()<< '\n';
    // std::cout << "_shank_FKRMP M: " << _shank_FKRMP->getM()<< '\n';

    // std::cout << "_foot_FKRMP f: " << _foot_FKRMP[1]->getF()<< '\n';
    // std::cout << "_foot_FKRMP M: " << _foot_FKRMP[1]->getM()<< '\n';
  // }

  _qddot.tail(_rootRMP->getDim()) = _rootRMP->getSolution();
  // std::cout << "q_ddot: " << _qddot << '\n';
  _tau_ff = (_A * _qddot + _coriolis + _grav).tail(6);
  // _q_des += _qdot_des*this->_data->userParameters->controller_dt;
  // _qdot_des += _qddot.block(6, 0, 6, 1)*this->_data->userParameters->controller_dt;

  for(int leg(0); leg<2; ++leg){
    this->_data->_legController->commands[leg].tauFeedForward = _tau_ff.block(3*leg, 0, 3, 1);
    // this->_data->_legController->commands[leg].qDes = _q_des.block(3*leg, 0, 3, 1);
    // this->_data->_legController->commands[leg].qdDes = _qdot_des.block(3*leg, 0, 3, 1);
    //
    // this->_data->_legController->commands[leg].kpJoint= 10.0*Mat3<float>::Identity();
    // this->_data->_legController->commands[leg].kdJoint= 0.3*Mat3<float>::Identity();

  }




  // auto* midSphere = this->_data->visualizationData->addSphere();
  // midSphere ->position =  this->_data->_stateEstimator->getResult().position + _collision_1;
  // midSphere ->radius = 0.05;
  // midSphere ->color = {0.8, 0.0, 0.0, 0.5};

}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_RMP<T>::onExit() {
  _iter = 0;
  _phase[0] = 0.0;
  _phase[1] = M_PI;
}

/**
 * Calculate the commands for the leg controllers for each of the feet.
 */
template <typename T>
void FSM_State_RMP<T>::BalanceStandStep() {
  UpdateGaitInfo();
  _iter++;
}
template <typename T>
T FSM_State_RMP<T>::SwingTrajectory(T phase, T swing_height){
  assert(phase>=0.0 && phase<=2*M_PI);
  /*
  Cubic Hermite Swing Trajectory
  */
  T z_ref = 0.0;
  T t;
  if(phase<M_PI/2){ //Swing up
    t  = (2.0/M_PI)*phase;
    z_ref = swing_height*(-2*pow(t, 3) + 3*pow(t, 2));
  }
  else if(phase<M_PI) //Swing Down
  {
    t  = (2.0/M_PI)*phase - 1;
    z_ref = swing_height*(2*pow(t, 3) - 3*pow(t, 2) + 1);
  }
  else{ //Stance
  }
  return z_ref;
}

template <typename T>
void FSM_State_RMP<T>::UpdateGaitInfo(){
  _phase[0] = 2*M_PI*(fmod(_iter*0.002, _gait_period)/_gait_period);//left leg
  _phase[0] = fmod(_phase[0], 2*M_PI);
  _phase[1] = fmod(_phase[0]+ M_PI, 2*M_PI);//right leg
  // _phase[1] = M_PI+0.01;
  Vec4<T> conphase;conphase.setZero();
  Vec4<T> contactState;contactState.setZero();

  for(size_t foot(0); foot < pat_biped::num_legs; foot++){
    conphase[foot] = _phase[foot]>M_PI? (_phase[foot]-M_PI)/ M_PI : 0.0;
    contactState[foot] = conphase[foot] > 0.0 ? 0.5 : 0.0;
  }


  this->_data->_stateEstimator->setContactPhase(conphase);
}
template <typename T>
bool FSM_State_RMP<T>::find_shank_capsule_witness_pts(Vec3<T>&wp1, Vec3<T>&wp2){
  Vec3<T>p1, p2, p3, p4;
  T cp1_r, cp2_r;
  cp1_r = 0.0;
  cp2_r = 0.0;
  p1 << _model.getPosition(pat_biped_linkID::RS, _shank_offsets[0][0]);
  p2 << _model.getPosition(pat_biped_linkID::RS, _shank_offsets[0][3]);
  p3 << _model.getPosition(pat_biped_linkID::LS, _shank_offsets[1][0]);
  p4 << _model.getPosition(pat_biped_linkID::LS, _shank_offsets[1][3]);




  bool wt_status = compute_witness_pts(p1, p2, p3, p4, cp1_r, cp2_r, wp1, wp2);
  if(wt_status){

    auto* w1Sphere = this->_data->visualizationData->addSphere();
    w1Sphere ->position << wp1;
    w1Sphere ->radius = 0.05;
    w1Sphere ->color = {1.0, 0.0, 0.0, 0.5};

    auto* w2Sphere = this->_data->visualizationData->addSphere();
    w2Sphere ->position << wp2;
    w2Sphere ->radius = 0.05;
    w2Sphere ->color = {0.0, 0.0, 1.0, 0.5};
    // std::cout << "wp1: "  << wp1-p1<< '\n';
    // std::cout << "global wp2: "  << wp2<< '\n';
    // std::cout << "global dist: " << (wp2-wp1).norm() << '\n';

  }
  else{

    std::cout << "couldnl't compute capsule witness points" << '\n';

  }
  return wt_status;

}
template <typename T>
bool FSM_State_RMP<T>::compute_witness_pts(Vec3<T>p1, Vec3<T>p2, Vec3<T>p3, Vec3<T>p4, T cp1_r, T cp2_r,
                                          Vec3<T>&wp1, Vec3<T>&wp2){
  //Input
  // p1, p2 p3, p4 pts defining two line segments,
  // cp1_r and cp2_r define the radius od the capsules
  //output
  // wp1 wp2 witness points on the capsules
  // https://homepage.univie.ac.at/franz.vesely/notes/hard_sticks/hst/hst.html

  Vec3<T> r1, r2, r12, e1, e2, s1, s2;
  T e12, e12_sqr, L1, L2;

  r1 = p1;
  r2 = p3;
  e1 = p2-p1;
  e2 = p4-p3;
  r12 = r2 - r1;


  DMat<T> G_eig; G_eig.resize(3,2);
  DVec<T> g0_eig(2); g0_eig.setZero();
  DMat<T> CE_eig(2,1); CE_eig.setZero();
  DVec<T> ce0_eig(1); ce0_eig.setZero();
  DMat<T> CI_eig(4, 2); CI_eig.setZero();
  DVec<T> ci0_eig(4); ci0_eig.setZero();


  GolDIdnani::GVect<double> z;
  // Cost
  GolDIdnani::GMatr<double> G;
  GolDIdnani::GVect<double> g0;

  // Equality
  GolDIdnani::GMatr<double> CE;
  GolDIdnani::GVect<double> ce0;

  // Inequality
  GolDIdnani::GMatr<double> CI;
  GolDIdnani::GVect<double> ci0;



  G_eig.block(0, 0, 3, 1) = -e1;
  G_eig.block(0, 1, 3, 1) = e2;
  g0_eig<<-2*r12.transpose()*e1, 2*r12.transpose()*e2;

  CI_eig<<1, 0,
      -1, 0,
      0, 1,
      0, -1;

  ci0_eig<<0, -1, 0, -1;

  G.resize(0., 2, 2);
  g0.resize(0., 2);
  CE.resize(0., 2, 0);
  ce0.resize(0., 0);
  CI.resize(0., 2, 4);
  ci0.resize(0., 4);


  auto H = 2*G_eig.transpose()*G_eig;
  eig_to_gold_mat(H, G, 2, 2);
  eig_to_gold_vec(g0_eig, g0, 2);
  eig_to_gold_mat(CI_eig.transpose(), CI, 2, 4);
  eig_to_gold_vec(-ci0_eig, ci0, 4);

  T f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
  (void)f;
  T mu1, mu2;
  mu1 = z[0];
  mu2 = z[1];
  s1 = r12 + mu2*e2 - mu1*e1;
  s2 = -r12 + mu1*e1 - mu2*e2;
  wp1 = r1 + mu1*e1 + cp1_r*s1/s1.norm();
  wp2 = r2 + mu2*e2 + cp2_r*s2/s2.norm();
  // std::cout << "z: " << z << '\n';
  // exit(0);
  return true;
}
template <typename T>
void FSM_State_RMP<T>::eig_to_gold_vec(DVec<T> eig, GolDIdnani::GVect<double> &gold, size_t size){
    for(size_t i(0); i<size; ++i){
      gold[i] = eig(i);
    }
}
template <typename T>
void FSM_State_RMP<T>::eig_to_gold_mat(DMat<T> eig, GolDIdnani::GMatr<double> &gold, size_t row, size_t col){
    for(size_t r(0); r<row; ++r){
      for(size_t c(0); c<col; ++c){
        gold[r][c] = eig(r, c);
      }
    }
}
// template class FSM_State_RMP<double>;
template class FSM_State_RMP<float>;
#endif
