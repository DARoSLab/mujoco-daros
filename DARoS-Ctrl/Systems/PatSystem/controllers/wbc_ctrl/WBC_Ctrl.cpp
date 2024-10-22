#include "WBC_Ctrl.hpp"
#include <Utilities/pretty_print.h>
#include <Utilities/Timer.h>
#include <ParamHandler/ParamHandler.hpp>

template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(const ControlFSMData<T> * data):
  _wbcLCM(getLcmUrl(255))
{
  _iter = 0;

  _data = data;
  _model = _data->_pat->buildModel();

  _full_config = DVec<T>(_model._nDof+1),
  _tau_ff = DVec<T>(_model._nDof-6),
  _des_jpos = DVec<T>(_model._nDof-6),
  _des_jvel = DVec<T>(_model._nDof-6),

  _kin_wbc = new KinWBC<T>(_model._nDof);

  _wbc = new WBIC<T>(_model._nDof, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<T>();

  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(pat_biped::num_legs_joint, 5.);
  _Kd_joint.resize(pat_biped::num_legs_joint, 1.5);
  _state.q = DVec<T>::Zero(_model._nDof-6);
  _state.qd = DVec<T>::Zero(_model._nDof-6);

  //_tauest= DVec<T>::Zero(pat_biped::dim_config);
  _tau_dist_old = DVec<T>::Zero(pat_biped::dim_config);
  _p_old = DVec<T>::Zero(pat_biped::dim_config);
  _CorMat = DMat<T>::Zero(pat_biped::dim_config,pat_biped::dim_config);

  _LoadJointLimitParams(THIS_COM"config/pat-joint-limit-parameters.yaml");
}

template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl(){
  delete _kin_wbc;
  delete _wbc;
  delete _wbic_data;

  typename std::vector<Task<T> *>::iterator iter = _task_list.begin();
  while (iter < _task_list.end()) {
    delete (*iter);
    ++iter;
  }
  _task_list.clear();

  typename std::vector<ContactSpec<T> *>::iterator iter2 = _contact_list.begin();
  while (iter2 < _contact_list.end()) {
    delete (*iter2);
    ++iter2;
  }
  _contact_list.clear();
}

template <typename T>
void WBC_Ctrl<T>::_ComputeWBC() {
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);
  _wbc->UpdateSetting(_A, _Ainv, _coriolis, _grav);
  _wbc->MakeTorque(_tau_ff, _wbic_data);
  // _wbc->FindConfigurationAndTorque(_full_config, _des_jpos, _des_jvel, _tau_ff, _wbic_data);
}

template <typename T>
void WBC_Ctrl<T>::_EstimateExternalForce(){
  Mat3<T> rot = ori::quaternionToRotationMatrix(_model._state.bodyOrientation);

  DVec<T> tau_full(_model._nDof); tau_full.setZero();
  tau_full.tail(_model._nDof-6) = _tau_ff;

  DMat<T> Jbody(3, _model._nDof); Jbody.setZero();
  Jbody.block(0, 3, 3, 3) = rot.transpose();
  DMat<T> Jbody_trans = Jbody.transpose();

  DMat<T> Jbody_trans_inv;
  pseudoInverse(Jbody_trans, 0.0001, Jbody_trans_inv);


  Vec18<T> qdotn;//from prev no n
  for (size_t i(0); i < 6; ++i) {
    qdotn[i] = _state.bodyVelocity[i];
  }
  for (size_t i(0); i < 18 - 6; ++i) {
    qdotn[i + 6] = _state.qd[i];
  }
  //T gamfil = 0.98;
  T gamfil = 0.284609544;

  //pretty_print(_Jc_full, std::cout, "Jc full");
  //pretty_print(_wbic_data->_Fr, std::cout, "Fr");

  DVec<T> negAqddot;
  if(_Jc_full.rows() >0){
    negAqddot = _coriolis - _grav + tau_full + _Jc_full.transpose()*_wbic_data->_Fr;
    //negAqddot = _coriolis - _grav + tau_full + _Jc_full.transpose()*_wbic_data->_Fr;
  }else{
    negAqddot = _coriolis - _grav + tau_full;
  }

  _tau_dist = gamfil*_tau_dist_old + (1-gamfil)*( (_A*qdotn - _p_old)/0.002f - negAqddot);

  _p_old=_A*qdotn;
  _tau_dist_old=_tau_dist;

  _F_ext_est = -Jbody_trans_inv * negAqddot;
  _F_ext_est_DOB = Jbody_trans_inv*_tau_dist;

  //_wbic_data->_F_ext = rot.transpose() * _F_ext_est_DOB;
/*

  // Option 2 ***************************
  DVec<T> Fr_sum = _grav; Fr_sum.setZero();
  for(int leg(0); leg<4; ++leg){
    int link_idx_= 3*leg+10;//new in Robot-Software
    Fr_sum += _model._Jc[link_idx_].transpose() * Fr.segment(3*leg, 3);
  }
  DVec<T> JtFext = _grav - Fr_sum - tau_full;


  DVec<T> F_ext = Jbody_trans_inv*JtFext;
  // Option 3 ***************************
  bool b_option_3(false);
  DVec<T> JtFr = _grav - tau_full;
  DMat<T> _Jc_full_tran = _Jc_full.transpose();
  DMat<T> _Jc_full_tran_inv;
  pseudoInverse(_Jc_full_tran, 0.0001, _Jc_full_tran_inv);
  DVec<T> Fr_full = _Jc_full_tran_inv*JtFr;
  Vec3<T> Fr_full_sum; Fr_full_sum.setZero();

  if(b_option_3){
    for (int i(0); i<3; ++i){
      for(int j(0); j<4; ++j){
        Fr_full_sum[i] += Fr_full[3*j + i];
      }
    }
  }

  // Option 4 ******************************
  bool b_option_4(false);
  if(b_option_4){
    DMat<T> Lambda_inv = _Jc_full * _Ainv * _Jc_full.transpose();
    DMat<T> Lambda = Lambda_inv.inverse();
    DMat<T> _Jc_full_bar = _Ainv * _Jc_full.transpose() * Lambda;
    DMat<T> Nc = DMat<T>::Identity(_model._nDof,_model._nDof) - _Jc_full_bar * _Jc_full;
    DMat<T> JbodyNc = Jbody*Nc;
    DMat<T> JN_trans = JbodyNc.transpose();
    DMat<T> JN_trans_inv;
    pseudoInverse(JN_trans, 0.0001, JN_trans_inv);
    //DVec<T> Fbody = JN_trans_inv*tau_full;
    DVec<T> Fbody = Nc.transpose()*(_grav - tau_full);

    // Option 5 ***************************
    DVec<T> JctransInvTau = _Jc_full_tran_inv*tau_full;
  }


  bool test_on(false);
  if(test_on){
    // Test 1 **************************
    DVec<T> tau_fr = tau_full.segment(6, 3);
    DMat<T> Jc_fr = _model._Jc[10];
    Vec3<T> Fr_fr = Fr.segment(0,3);
    DMat<T> Jc_fr_trans = Jc_fr.transpose();
    DMat<T> Jc_fr_trans_inv;
    pseudoInverse(Jc_fr_trans, 0.00001, Jc_fr_trans_inv);
    //Vec3<T> Fr_fr_est = Jc_fr_trans_inv*tau_fr;
    Vec3<T> Fr_fr_est = Jc_fr_trans_inv*tau_full;

    // Test 2 *******************
    DMat<T> Jc_fr_frac = Jc_fr.block(0,6, 3,3);
    DMat<T> Jc_fr_frac_trans = Jc_fr_frac.transpose();
    DMat<T> Jc_fr_frac_trans_inv = Jc_fr_frac_trans.inverse();

    //DMat<T> test_mtx = Jc_fr_frac_trans_inv*Jc_fr_frac_trans;
    Vec3<T> Fr_fr_frac_est = Jc_fr_frac_trans_inv*tau_fr;
  }
*/
  // **************************************************************************
  static size_t iter(0);
  ++iter;
  if(iter%100 == 20){
    //pretty_print(Jc_fr_trans_inv, std::cout, "Jc_trans_inv");
    //pretty_print(Jc_fr_frac_trans, std::cout, "Jc_fr_frac_trans");
    //pretty_print(Jc_fr_frac_trans_inv, std::cout, "Jc_fr_frac_trans_inv");
    //pretty_print(test_mtx, std::cout, "test");

    //pretty_print(tau_fr, std::cout, "tau_fr");
    //pretty_print(Fr_fr, std::cout, "Fr_fr");
    //pretty_print(Jc_fr, std::cout, "Jc_fr");
    //pretty_print(Fr_fr_est, std::cout, "Fr_fr est");
    //pretty_print(Fr_fr_frac_est, std::cout, "Fr_frac_fr est");
    //pretty_print(JctransInvTau, std::cout, "Jctran inv tau");


    //pretty_print(Fbody, std::cout, "Fbody");

    //pretty_print(Fr_full, std::cout, "Fr_full");
    //pretty_print(Fr_full_sum, std::cout, "Fr_full_sum");



    //pretty_print(F_ext, std::cout, "Fext");
    //pretty_print(sum_Fr, std::cout, "Fr_sum");
    //pretty_print(_tau_ff, std::cout, "tau_full");
    //pretty_print(_taudist, std::cout, "tau_disturbance");
    //pretty_print(tau_adapt, std::cout, "tau_full");
    //pretty_print(est_f, std::cout, "estimated_force");
    //pretty_print(est_f_gabe, std::cout, "estimated_force Gabe");
    //pretty_print(lin_compensation, std::cout, "lin_compensation");
    //pretty_print(new_est_f, std::cout, "lin_comp + est_f");

    //pretty_print(lin_grav, std::cout, "global lin gravity");
    //pretty_print(_grav, std::cout, "gravity");
    //pretty_print(_coriolis, std::cout, "coriolis");
    //pretty_print(sum, std::cout, "sum of grav cori");
    //pretty_print(_A, std::cout, "Mass");
    //printf("\n");
  }
}

template<typename T>
void WBC_Ctrl<T>::run(void* input, ControlFSMData<T> & data){
  ++_iter;
  // Update Model
  _UpdateModel();
  // _UpdateModel_local();

  // Task & Contact Update
  _ContactTaskUpdate(input);

  // WBC Computation
  _ComputeWBC();

  // Update Leg Command
  _UpdateLegCMD();

  // Update state est. values from WBC calculations
  //_UpdateStateEst(data);

  // LCM publish
  _LCM_PublishData();

  //_EstimateContactForces();//Comment if not needed

  //_EstimateExternalForce();
}

template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(){
  LegControllerPatCommand<T> * cmd = _data->_legController->commands;
  auto datas = _data->_legController->datas;
  // for(int leg(0); leg<2; ++leg){
  //
  //   for(int i(0); i<3; ++i){
  //     if(datas[leg]->q[i]<_jpos_min[3*leg + i] || datas[leg]->q[i]>_jpos_max[3*leg + i]){
  //       if(!ESTOP){
  //
  //         std::cout << "[WBC ESTOP] leg " << leg <<" joint " << i<< " limit violated" << '\n';
  //         std::cout << "joint pos: " << datas[leg]->q[i] << '\n';
  //         std::cout << "joint min: " << _jpos_min[3*leg + i] << '\n';
  //         std::cout << "joint max: " << _jpos_max[3*leg + i] << '\n';
  //         ESTOP = true;
  //       }
  //
  //     }
  //
  //   }
  //
  // }
  // auto a_r = _data->_legController->datas[0]->q[0];
  // auto a_l = _data->_legController->datas[1]->q[0];
  // auto h_r = _data->_legController->datas[0]->q[1];
  // auto h_l = _data->_legController->datas[1]->q[1];
  // auto k_r = _data->_legController->datas[0]->q[2];
  // auto k_l = _data->_legController->datas[1]->q[2];
  // if(a_r < -0.15 || a_r > 0.6 ||  a_l < -0.6 || a_l > 0.15){
  //   if(!ESTOP){
  //
  //     std::cout << "[WBC ESTOP Abd joint limit]" << '\n';
  //     std::cout << "a_l: " <<a_l << '\n';
  //     std::cout << "a_r: " <<a_r << '\n';
  //     ESTOP = true;
  //
  //   }
  //
  // }
  // if(h_r < -0.9 || h_r > -0.15 ||  h_l < -0.9 || h_l > -0.15){
  //   if(!ESTOP){
  //     std::cout << "[WBC ESTOP Hip joint limit]" << '\n';
  //     std::cout << "h_l: " <<h_l << '\n';
  //     std::cout << "h_r: " <<h_r << '\n';
  //     ESTOP = true;
  //
  //   }
  //
  // }
  // if(k_r < 0.6 || k_r > 1.6 ||  k_l < 0.6 || k_l > 1.6){
  //   if(!ESTOP){
  //
  //     std::cout << "[WBC ESTOP Knee joint limit]" << '\n';
  //     std::cout << "k_l: " <<k_l << '\n';
  //     std::cout << "k_r: " <<k_r << '\n';
  //     ESTOP = true;
  //
  //   }
  //
  // }

  for(int foot=0; foot<2; foot++){
    size_t gcID = _model._footIndicesGC.at(foot);
    auto foot_position = _model._pGC.at(gcID);
    // if(!ESTOP && pow(-1, foot)*(base_position[1]-foot_position[1])<0.01){
    //   std::cout << "[WBC ESTOP Foot Collision]" << '\n';
    //   ESTOP = true;
    //
    // }
    if(!ESTOP && foot_position[2] > 0.1){

      std::cout << "[WBC ESTOP Foot Height]" << '\n';
      ESTOP = true;

    }
  }
  Vec3<float> rpy, bp;
  rpy = _data->_stateEstimator->getResult().rpy;
  bp = _data->_stateEstimator->getResult().position;

  if(!ESTOP && (rpy.block<2, 1>(0, 0).norm()>0.85 || bp[2]<0.25))
  {
    std::cout << "[WBC ESTOP BASE]" << '\n';
    ESTOP = true;
  }
  for (size_t leg(0); leg < pat_biped::num_legs; ++leg)
      cmd[leg].zero(); // Disable the previous leg control command

  if(!ESTOP){
    for (size_t leg(0); leg < pat_biped::num_legs; ++leg) {
        // cmd[leg].zero(); // Disable the previous leg control command
        for (size_t jidx(0); jidx < pat_biped::num_legs_joint; ++jidx) {
          cmd[leg].tauFeedForward[jidx] = _tau_ff[pat_biped::num_legs_joint * leg + jidx];
          cmd[leg].qDes[jidx] = _des_jpos[pat_biped::num_legs_joint * leg + jidx];
          cmd[leg].qdDes[jidx] = _des_jvel[pat_biped::num_legs_joint * leg + jidx];

          cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
          cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];

          //pretty_print(_Kp_joint, "Kp_joint");
          //pretty_print(_Kp_joint, "Kd_joint");
          //cmd[leg].kpJoint(jidx, jidx) = 0.;
          //cmd[leg].kdJoint(jidx, jidx) = 0.;
        }
      }
  }
  else{
    if (_iter%1000==0)
      std::cout << "[WBC] ESTOP!" << '\n';
  }
  // Knee joint non flip barrier
  // for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
  //   LimbData<T>* leg_data = _data->_legController->datas[leg];
  //
  //   if(cmd[leg].qDes[2] < 0.2){
  //     cmd[leg].qDes[2] = 0.2;
  //   }
  //   if(leg_data->q[2] < 0.2){
  //     T knee_pos = leg_data->q[2];
  //     if(knee_pos>0.){
  //       cmd[leg].tauFeedForward[2] = 1./(knee_pos * knee_pos + 0.04);
  //     }else{
  //       cmd[leg].tauFeedForward[2] = 20.;
  //     }
  //   }
  // }
}

template<typename T>
void WBC_Ctrl<T>::_UpdateModel(){

  const StateEstimate<T> & state_est = _data->_stateEstimator->getResult();
  LimbData<T> * const * leg_data = _data->_legController->datas;
  if (first_visit){
		rpy_ini = ori::quatToRPY(_data->_stateEstimator->getResult().orientation);
    rpy_ini[0] = 0;
    rpy_ini[1] = 0;
    _ori_ini_inv = ori::rpyToQuat(-rpy_ini);
    first_visit =false;
    // std::cout << "rpy_ini: wbc " << rpy_ini << '\n';
 		// first_visit is set to "false" in getFootLocations
	}
	//rpy[2] -= rpy_ini[2];
	//state.bodyOrientation = ori::rpyToQuat(rpy);

	// Create robot model
  // Remove yaw offset
  Mat3<T> R_yaw_offset = ori::quaternionToRotationMatrix(_ori_ini_inv).transpose();

  _state.bodyOrientation =
    ori::quatProduct(_ori_ini_inv, state_est.orientation);
  // _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = R_yaw_offset*state_est.position;
  _state.bodyVelocity.head(3) = state_est.omegaBody;
  _state.bodyVelocity.tail(3) = state_est.vBody;
  for(size_t i(0); i<3; ++i){
    for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
      _state.q[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->q[i];
      _state.qd[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->qd[i];

      _full_config[3*leg + i + 6] = _state.q[3*leg + i];
      //_tauest(3*leg + i + 6)=((LegControllerPatData<T>*)leg_data[leg])->tauEstimate[i];//Comment if not needed
    }
  }
  _model.setState(_state);

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

  //_EstimateContactForces();

  //_grav.setZero();
  //pretty_print(_A, std::cout, "A");
}

template<typename T>
void WBC_Ctrl<T>::_UpdateModel_local(){

  const StateEstimate<T> & state_est = _data->_stateEstimator->getResult();
  LimbData<T> * const * leg_data = _data->_legController->datas;
  if (first_visit){
		rpy_ini = ori::quatToRPY(state_est.orientation);
    rpy_ini[0] = 0;
    rpy_ini[1] = 0;
    _ori_ini_inv = ori::rpyToQuat(-rpy_ini);
    first_visit =false;
	}

  _state.bodyOrientation =
    ori::quatProduct(_ori_ini_inv, state_est.orientation);
  _state.bodyVelocity.head(3) = state_est.omegaBody;

  _state.bodyPosition.setZero();
  _state.bodyVelocity.tail(3).setZero();

  for(size_t i(0); i<3; ++i){
    for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
      _state.q[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->q[i];
      _state.qd[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->qd[i];

      _full_config[3*leg + i + 6] = _state.q[3*leg + i];
      //_tauest(3*leg + i + 6)=((LegControllerPatData<T>*)leg_data[leg])->tauEstimate[i];//Comment if not needed
    }
  }
  _model.setState(_state);
  _model.forwardKinematics();
  int linkid =  state_est.contactEstimate(0) > 0.0 ? pat_biped_linkID::RF : pat_biped_linkID::LF;
  _state.bodyPosition = -_model._pGC[linkid];
  _state.bodyVelocity.tail(3) = -_model._vGC[linkid];

  _model.setState(_state);

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

  //_EstimateContactForces();

  //_grav.setZero();
  //pretty_print(_A, std::cout, "A");
}
template<typename T>
void WBC_Ctrl<T>::_EstimateContactForces(){
  _model.CoriolisMatrixonly();//here to avoid delaying the WBC QP computation
  _CorMat = _model.getCorMatrix();
  Vec18<T> qdotn;//from prev no n
  for (size_t i(0); i < 6; ++i) {
    qdotn[i] = _state.bodyVelocity[i];
  }
  for (size_t i(0); i < 18 - 6; ++i) {
    qdotn[i + 6] = _state.qd[i];
  }
  //T gamfil = 0.151835802f;//0.284609544f;//0.828614707f;
  //T gamfil = 0.284609544;
  //T gamfil = 0.828614707f;
  T gamfil = 0.98;
  //_taudist=gamfil*_taudistold+(1-gamfil)*((_A*qdotn-_pold)/0.002f-(_CorMat.transpose()*qdotn-_grav+_tauest));


  DVec<T> tau_full(_model._nDof); tau_full.setZero(); tau_full.tail(_model._nDof - 6) = _tau_ff;

  //pretty_print(_tauest, std::cout, "tau est");
  //pretty_print(tau_full, std::cout, "tau full");


  //DVec<T> coriolis_mtx = _CorMat * qdotn;
  //pretty_print(coriolis_mtx, std::cout, "cori mtx");
  //pretty_print(_coriolis, std::cout, "coriolis");

  _tau_dist = gamfil*_tau_dist_old +
    (1-gamfil)*( (_A*qdotn - _p_old)/0.002f - (_coriolis - _grav + tau_full + _Jc_full.transpose()*_wbic_data->_Fr));
  //VERIFY THAT CONTACT JACOBIAN DOES NOT INTERFERE IN THE DYNAMICS
  //cout<<"_taudist \t"<<_taudist.transpose()<<endl; HI2
  //Timer timer_legest;
  Eigen::Matrix<T,18,12> Jsim;
  for(size_t leg = 0; leg<pat_biped::num_legs; ++leg){
    int link_idx_= 3*leg+10;//new in Robot-Software
    Jsim.block(0, 3*leg, 18, 3) = _model._Jc[link_idx_].transpose();
    //Vec18<T>_tausol=_taudist;//.block(3*leg+6,0,3,1)
  }
  forsol =  Jsim.colPivHouseholderQr().solve(_tau_dist);

  //pretty_print(forsol, std::cout, "f contact");
  //pretty_print(_wbic_data->_Fr, std::cout, "Fr WBIC");
  /*   // Gabriel's landing conditions
  trigland = false;
  for(int leg = 0; leg<4; ++leg){
    if (forsol(3*leg+2)>20.f){//define a better condition for "contact" (e.g. probabilistics, machine learning, etc.).
      int link_idx_= 3*leg+10;
      cout<<"possible contact at leg["<<leg<<"] with Force: "<<forsol.block(3*leg,0,3,1).transpose()
        <<"\t leg new speed: "<< _model._vGC[link_idx_].transpose()<<endl;
      trigland = true;
    }
  }
  */
  //printf("\t Timer: %.3f ms., \t", timer_legest.getMs());
  //cout<<endl;
  //cout<< "_tauest: "<<_tauest.transpose()<<endl;
  //cout<< "_taudist: "<<_taudist.transpose()<<endl;
  //cout<< "forces: "<<forsol.transpose();


  /*Timer timer_legest;
  for(int leg = 0; leg<4; ++leg){
    int link_idx_= 2*leg+9;
    Eigen::Matrix<T,3,3> Jsim = _model._Jc[link_idx_].block(0, 3*leg+6, 3, 3).transpose();
    Vec3<T>_tausol=_taudist.block(3*leg+6,0,3,1);
    Vec3<T> forsol =  Jsim.ldlt().solve(_tausol);
    cout<< "leg["<<leg<<"]: "<<forsol.transpose();
    //pseudoInverse(Wpla,0.001,Wplainv);
  }
  printf("\t Timer: %.3f ms.", timer_legest.getMs());
*/
  /*Timer timer_legest;
  for(int leg = 0; leg<4; leg++){
    Vec6<T>_tausol;
    int link_idx_= 2*leg+9;
    Eigen::Matrix<T,6,3> Jsim;
    Jsim.block(0,0,3,3) = _model._Jc[link_idx_].block(0, 3, 3, 3).transpose();
    Jsim.block(3,0,3,3) = _model._Jc[link_idx_].block(0, 3*leg+6, 3, 3).transpose();
    _tausol.block(0,0,3,1)=_taudist.block(3,0,3,1);
    _tausol.block(3,0,3,1)=_taudist.block(3*leg+6,0,3,1);
    Vec3<T> forsol =  Jsim.colPivHouseholderQr().solve(_tausol);//householderQr
    cout<< "link_idx_"<< link_idx_;
    cout<< "Jsim"<< Jsim<< endl;
    cout<< "_tausol"<< _tausol<<endl;
    cout<< "leg["<<leg<<"]: "<<forsol.transpose();
    //pseudoInverse(Wpla,0.001,Wplainv);
  }
  printf("\t Timer: %.3f ms.", timer_legest.getMs());*/
  _p_old=_A*qdotn;
  _tau_dist_old=_tau_dist;
}
template <typename T>
void WBC_Ctrl<T>::_LoadJointLimitParams(const std::string& file){

  ParamHandler handler(file);
  handler.getVector("jpos_min", _jpos_min);
  handler.getVector("jpos_max", _jpos_max);
  printf("[WBC] Joint Limit Parameter Setup is completed\n");

}
template class WBC_Ctrl<float>;
//template class WBC_Ctrl<double>;
