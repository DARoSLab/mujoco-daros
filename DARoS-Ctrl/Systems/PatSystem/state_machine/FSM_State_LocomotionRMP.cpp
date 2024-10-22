/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */
#ifdef RMPFLOW_BUILD
#include "FSM_State_LocomotionRMP.h"
#include <Utilities/Timer.h>
#include <wbc_ctrl/LocomotionCtrlRMP/LocomotionCtrlRMP.hpp>
#include <common/control/RMPFlow/LeafNodeSet/LinkAttractorRMP.hpp>
#include <common/control/RMPFlow/LeafNodeSet/LinkCollisionRMP.hpp>
#include <common/control/RMPFlow/LeafNodeSet/LinkFKRMP.hpp>
#include <common/control/RMPFlow/LeafNodeSet/ContactFKRMP.hpp>
#include <fstream>
//#include <rt/rt_interface_lcm.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_LocomotionRMP<T>::FSM_State_LocomotionRMP(ControlFSMData<T>* _controlFSMData)
  : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  _wbc_ctrl = new LocomotionCtrlRMP<T>(_controlFSMData);
  _wbc_data = new LocomotionCtrlRMPData<T>();
  _wbc_ctrl->setFloatingBaseWeight(1000.);
  // _model = this->_data->_pat->buildModel();

  _model = _wbc_ctrl->getModelPtr();
  _rmp_model = this->_data->_pat->buildModel();
  _rmp_state.q = DVec<T>::Zero(_model->_nDof-6);
  _rmp_state.qd = DVec<T>::Zero(_model->_nDof-6);
  #ifdef USE_CMPC
    cMPC_biped = new cMPC_BipedLocomotion(_controlFSMData->userParameters->controller_dt,
      (int)33 / (1000. * _controlFSMData->userParameters->controller_dt),
      _controlFSMData->userParameters, _model);
      std::cout << "[LOCOMOTION] Using MPC" << '\n';
  #else
    std::cout << "[LOCOMOTION] Using WBC" << '\n';
    cMPC_biped = new WBC_BipedLocomotion(_controlFSMData->userParameters, _model);
  #endif
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF and footstep locations to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();
  _shank_offsets[0][0]<<0.0, 0.0, 0.0;
  _shank_offsets[0][1]<<0.0, -0.004, -0.1;
  _shank_offsets[0][2]<<0.0, -0.004, -0.15;
  _shank_offsets[0][3]<<0.0, -0.004, -0.20;
  // _shank_offsets[1][0]<<0.0, 0, 0;
  _shank_offsets[1][0]<<0.0, 0.0, 0.0;
  _shank_offsets[1][1]<<0.0, 0.004, -0.1;
  _shank_offsets[1][2]<<0.0, 0.004, -0.15;
  _shank_offsets[1][3]<<0.0, 0.004, -0.20;


}

template <typename T>
void FSM_State_LocomotionRMP<T>::onEnter() {
  cMPC_biped->initialize();
  iter = 0;
  printf("[FSM RMP LOCOMOTION] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_LocomotionRMP<T>::run() {
  updateRMPState();
  LocomotionControlStep();
  ++iter;
}


/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_LocomotionRMP<T>::onExit() {
  // Nothing to clean up when exiting
  iter = 0;
}

template <typename T>
void FSM_State_LocomotionRMP<T>::updateRMPState(){
  const StateEstimate<T> & state_est = this->_data->_stateEstimator->getResult();
  LimbData<T> * const * leg_data = this->_data->_legController->datas;
  if (first_visit){
		rpy_ini = ori::quatToRPY(this->_data->_stateEstimator->getResult().orientation);
    rpy_ini[0] = 0;
    rpy_ini[1] = 0;
    _ori_ini_inv = ori::rpyToQuat(-rpy_ini);
    first_visit =false;
	}

  Mat3<T> R_yaw_offset = ori::quaternionToRotationMatrix(_ori_ini_inv).transpose();

  _rmp_state.bodyOrientation =
    ori::quatProduct(_ori_ini_inv, state_est.orientation);
  // _state.bodyOrientation = state_est.orientation;
  _rmp_state.bodyPosition = R_yaw_offset*state_est.position;
  _rmp_state.bodyVelocity.head(3) = state_est.omegaBody;
  _rmp_state.bodyVelocity.tail(3) = state_est.vBody;

  for(int leg(0); leg<2; ++leg){

    for (int i(0);i < 3;++i){
      _rmp_state.q[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->q[i];
      _rmp_state.qd[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->qd[i];

    }

  }
  _rmp_model.setState(_rmp_state);
  _rmp_model.forwardKinematics();
  _rmp_model.contactJacobians();
  _rmp_model.massMatrix();//Comment if using massandCoriolisMatrix()
  _rmp_model.generalizedGravityForce();
  _rmp_model.generalizedCoriolisForce();
  _rmp_model.centroidMomentumMatrix();

}
/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_LocomotionRMP<T>::LocomotionControlStep() {

  cMPC_biped->run(*this->_data);

  _wbc_data->pBody_des = cMPC_biped->pBody_des.template cast<T>();
  _wbc_data->vBody_des = cMPC_biped->vBody_des.template cast<T>();
  _wbc_data->aBody_des = cMPC_biped->aBody_des.template cast<T>();

  _wbc_data->pBody_RPY_des = cMPC_biped->pBody_RPY_des.template cast<T>();
  _wbc_data->vBody_Ori_des = cMPC_biped->vBody_Ori_des.template cast<T>();
  _wbc_data->pBody_des[0] += _wbc_data->vBody_des[0]*(this->_data->userParameters->controller_dt);
  _wbc_data->pBody_des[1] += _wbc_data->vBody_des[1]*(this->_data->userParameters->controller_dt);

  for(size_t i(0); i<pat_biped::num_legs; ++i){
  _wbc_data->pFoot_des[i] = cMPC_biped->pFoot_des[i];
  _wbc_data->vFoot_des[i] = cMPC_biped->vFoot_des[i];
  _wbc_data->aFoot_des[i] = cMPC_biped->aFoot_des[i];
  _wbc_data->Fr_des[i] << 0.0, 0.0, 0.0; //cMPC_biped->Fr_des[i];
  }

  _wbc_data->pBody_des = cMPC_biped->pBody_des.template cast<T>();
  _wbc_data->contact_state = cMPC_biped->contact_state;

  Vec3<T>p1, p2, p3, p4;

  p1 << _rmp_model.getPosition(pat_biped_linkID::RS, _shank_offsets[0][0]);
  p2 << _rmp_model.getPosition(pat_biped_linkID::RS, _shank_offsets[0][3]);
  p3 << _rmp_model.getPosition(pat_biped_linkID::LS, _shank_offsets[1][0]);
  p4 << _rmp_model.getPosition(pat_biped_linkID::LS, _shank_offsets[1][3]);

  _rootRMP = new RMPRoot<float>("Root", _model->_nDof, &_rmp_model);

  for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
    if(_wbc_data->contact_state[leg] > 0.){ // Contact

    }else{ // No Contact (swing)


      int link_id = leg == 0 ? pat_biped_linkID::RF : pat_biped_linkID::LF;
      Vec3<T> _foot_offset; _foot_offset << 0.0, 0.0, 0.0;
      _foot_FKRMP[leg] = new ContactFKRMP<float>("FootFK_" + std::to_string(leg), 3, _rootRMP, &_rmp_model, link_id, _foot_offset);
      _foot_attractorRMP[leg] = new LinkAttractorRMP<float>("FootAttractor_" + std::to_string(leg), 3, _foot_FKRMP[leg], link_id);

      DVec<T> des_pos_vel_acc = DVec<T>::Zero(9);
      des_pos_vel_acc << _wbc_data->pFoot_des[leg], _wbc_data->vFoot_des[leg], _wbc_data->aFoot_des[leg];
      _foot_attractorRMP[leg]->update_parameters(des_pos_vel_acc);

      Vec3<T> wp1;
      Vec3<T> wp2;

      bool wps = find_shank_capsule_witness_pts(wp1, wp2);
      if(wps){




        _collision_loc = leg ==0 ? wp2:wp1;
        _control_loc = leg ==1 ? wp2:wp1;

        auto* w1Sphere = this->_data->visualizationData->addSphere();
        w1Sphere ->position << _collision_loc;
        w1Sphere ->radius = 0.05;
        w1Sphere ->color = {1.0, 1.0, 0.0, 0.5};

        auto* w2Sphere = this->_data->visualizationData->addSphere();
        w2Sphere ->position << _control_loc;
        w2Sphere ->radius = 0.05;
        w2Sphere ->color = {0.0, 1.0, 1.0, 0.5};


        Vec3<T> shank_offset = leg ==0 ? wp1-p1:wp2-p3;
        int s_link_id = leg == 0 ? pat_biped_linkID::RS : pat_biped_linkID::LS;
        Mat4<T> W_T_L;
        W_T_L.block(0, 0, 3, 3) <<_rmp_model.getOrientation(s_link_id); // Link orientation in world
        W_T_L.block(0, 3, 3, 1) <<_rmp_model.getPosition(s_link_id);
        W_T_L(3, 3) = 1.0;
        Vec4<T> shank_offset_h;
        shank_offset_h << shank_offset, 1.0;
        shank_offset = (W_T_L.transpose()*shank_offset_h).block(0,0, 3, 1);//offset in link frame

        _shank_FKRMP = new LinkFKRMP<float>("shankFK_" + std::to_string(leg), 3, _rootRMP, &_rmp_model, s_link_id, shank_offset);
        _shank_collisionRMP = new LinkCollisionRMP<float>("shankCollision" + std::to_string(0), 1, _shank_FKRMP, 5);
        _shank_collisionRMP->update_parameters(_collision_loc);
        // if(iter<1000)
        //   _shank_collisionRMP->setMu(0.0);
        // else
        //   _shank_collisionRMP->setMu(80.0);

        // (void)_collision_loc;
      }

    }
  }

 // std::cout << "robot id: " << this->_data->_robot_id << '\n';
 if(iter > 3000){
   std::cout << "Done!" << '\n';
   dump_exit_status();
   exit(0);
 }
  _rootRMP->solve();
  //
  DMat<T>M = _rootRMP->getM();
  DVec<T>f = _rootRMP->getF();
  for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
    if(_wbc_data->contact_state[leg] > 0.){ // Contact

    }else{ // No Contact (swing)

    // Vec3<T> position; position<<_control_loc;
    // Vec3<T> direction; direction<<_shank_FKRMP->getF();
    // T color[3] = {0.0, 1.0, 0.0};
    // T brightness = 0.7;
    // T scale = 1.0;
    // this->VisualizeArrow(position, direction, color, brightness, scale);

    // std::cout << "Foot Attractor F: " << _foot_FKRMP[leg]->getF() << '\n';
    // std::cout << "Shank Collision F: " << _shank_FKRMP->getF() << '\n';

    // std::cout << "Foot Attractor M: " << _foot_FKRMP[leg]->getM() << '\n';
    // std::cout << "Shank Collision M: " << _shank_FKRMP->getM() << '\n';
    // std::cout << "Collision distance: " << _shank_collisionRMP->getX() << '\n';
    _rmp_lcm.collision_pt[0] = _collision_loc[0];
    _rmp_lcm.collision_pt[1] = _collision_loc[1];
    _rmp_lcm.collision_pt[2] = _collision_loc[2];

    _rmp_lcm.control_pt[0] = _control_loc[0];
    _rmp_lcm.control_pt[1] = _control_loc[1];
    _rmp_lcm.control_pt[2] = _control_loc[2];

    // _rmp_lcm.collision_F[0] = (T)_shank_FKRMP->getF().norm();
    _rmp_lcm.goal_F[0] = (T)_foot_FKRMP[leg]->getF().norm();
    // _rmp_lcm.collision_F[1] = _foot_FKRMP[leg]->getF()[1];
    // _rmp_lcm.collision_F[2] = _foot_FKRMP[leg]->getF()[2];
    T distance = sqrt((double)((_collision_loc - _control_loc).transpose()*(_collision_loc - _control_loc)));
    if(distance < _min_distance) _min_distance = distance;
    _rmp_lcm.distance = distance;
    }
  }
  _rmpLCM.publish("rmp_lcmt", &_rmp_lcm);
  // std::cout << "RMP Soln: " << _rootRMP->getSolution() << '\n';
  // std::cout << "RMP M: " << M << '\n';
  // std::cout << "RMP f: " << f << '\n';

  _wbc_ctrl->updateRMP(M, f);

  _wbc_ctrl->run(_wbc_data, *this->_data);

  Vec4<T> se_contactState; se_contactState.setZero();
  se_contactState << cMPC_biped->contact_state, 0, 0;
  this->_data->_stateEstimator->setContactPhase(se_contactState);
}
template <typename T>
void FSM_State_LocomotionRMP<T>::dump_exit_status(){
  bool exit_status = _rmp_state.bodyPosition[2] > 0.3;
  ofstream myfile;
  myfile.open(THIS_COM "config/exit_statuses/"+ std::to_string(this->_data->_robot_id) + ".txt");
  myfile << exit_status;
  myfile << "\n";
  myfile << _min_distance;
  myfile.close();
  printf("Robot %d Done\n", this->_data->_robot_id);
}
template <typename T>
bool FSM_State_LocomotionRMP<T>::find_shank_capsule_witness_pts(Vec3<T>&wp1, Vec3<T>&wp2){
  Vec3<T>p1, p2, p3, p4;
  T cp1_r, cp2_r;
  cp1_r = 0.0;
  cp2_r = 0.0;
  p1 << _rmp_model.getPosition(pat_biped_linkID::RS, _shank_offsets[0][0]);
  p2 << _rmp_model.getPosition(pat_biped_linkID::RS, _shank_offsets[0][3]);
  p3 << _rmp_model.getPosition(pat_biped_linkID::LS, _shank_offsets[1][0]);
  p4 << _rmp_model.getPosition(pat_biped_linkID::LS, _shank_offsets[1][3]);




  bool wt_status = compute_witness_pts(p1, p2, p3, p4, cp1_r, cp2_r, wp1, wp2);

  return wt_status;

}
template <typename T>
bool FSM_State_LocomotionRMP<T>::compute_witness_pts(Vec3<T>p1, Vec3<T>p2, Vec3<T>p3, Vec3<T>p4, T cp1_r, T cp2_r,
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
void FSM_State_LocomotionRMP<T>::eig_to_gold_vec(DVec<T> eig, GolDIdnani::GVect<double> &gold, size_t size){
    for(size_t i(0); i<size; ++i){
      gold[i] = eig(i);
    }
}
template <typename T>
void FSM_State_LocomotionRMP<T>::eig_to_gold_mat(DMat<T> eig, GolDIdnani::GMatr<double> &gold, size_t row, size_t col){
    for(size_t r(0); r<row; ++r){
      for(size_t c(0); c<col; ++c){
        gold[r][c] = eig(r, c);
      }
    }
}

// template class FSM_State_LocomotionRMP<double>;
template class FSM_State_LocomotionRMP<float>;
#endif
