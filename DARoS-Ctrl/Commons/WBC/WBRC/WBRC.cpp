#include "WBRC.hpp"
#include <Utilities/Timer.h>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#define PRIORITY_CD
template <typename T>
WBRC<T>::WBRC(size_t num_qdot, const std::vector<ContactSpec<T>*>* contact_list, const std::vector<Task<T>*>* task_list) : WBC<T>(num_qdot), _dim_floating(6) {
    _contact_list = contact_list;
    _task_list = task_list;
    _eye = DMat<T>::Identity(WBC<T>::num_qdot_, WBC<T>::num_qdot_);
}

template <typename T>
void WBRC<T>::MakeTorqueWBRC(DVec<T>& cmd, void* extra_input) {
  if (!WBC<T>::b_updatesetting_) printf("[Warning] WBRC setting is not done\n");
  if (extra_input) wbc_data = static_cast<WBIC_ExtraData<T>*>(extra_input);
  _SetOptimizationSize(); // resize G, g0, CE, ce0, CI, ci0
  _SetCost();
  DVec<T> qddot_pre, JtDotQdot, xddot;
  DMat<T> Npre, Jt, JtBar, JtPre;
  Task<T>* task;
  //cout<<"yei1"<<endl;
#ifdef PRIORITY_CD
  qddot_pre = DVec<T>::Zero(WBC<T>::num_qdot_);
  Npre = _eye;
  task = (*_task_list)[0];
  task->getTaskJacobian(Jt);
  task->getTaskJacobianDotQdot(JtDotQdot);
  task->getCommand(xddot);
  JtPre = Jt * Npre;
  WBC<T>::_WeightedInverse(JtPre, WBC<T>::Ainv_, JtBar);
  qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
  Npre = Npre * (_eye - JtBar * JtPre);
  if (_dim_rf > 0) {
    _ContactBuilding();// Contact Setting
    _SetInEqualityConstraint();// Set inequality constraints
    JtPre = _Jc * Npre;
    WBC<T>::_WeightedInverse(JtPre, WBC<T>::Ainv_, JtBar);
    qddot_pre += JtBar * (- _JcDotQdot - _Jc * qddot_pre);
    Npre = Npre * (_eye - JtBar * JtPre);
    // pretty_print(JtBar, std::cout, "JtBar");
    // pretty_print(_JcDotQdot, std::cout, "JcDotQdot");
    // pretty_print(qddot_pre, std::cout, "qddot 1");
  }
#else
  if (_dim_rf > 0) {
    DMat<T> JcBar;
    _ContactBuilding();// Contact Setting
    _SetInEqualityConstraint();// Set inequality constraints
    WBC<T>::_WeightedInverse(_Jc, WBC<T>::Ainv_, JcBar);
    qddot_pre = JcBar * (-_JcDotQdot);//how about enhancing...?
    Npre = _eye - JcBar * _Jc;
    // pretty_print(JcBar, std::cout, "JcBar");
    // pretty_print(_JcDotQdot, std::cout, "JcDotQdot");
    // pretty_print(qddot_pre, std::cout, "qddot 1");
  } else {
    qddot_pre = DVec<T>::Zero(WBC<T>::num_qdot_);
    Npre = _eye;
  }
#endif
  //cout<<"yei2"<<endl;
#ifdef PRIORITY_CD
  for (size_t i(1); i < (*_task_list).size(); ++i) {//0
#else
  for (size_t i(0); i < (*_task_list).size(); ++i) {//0
#endif
    task = (*_task_list)[i];
    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    task->getCommand(xddot);
    JtPre = Jt * Npre;
    WBC<T>::_WeightedInverse(JtPre, WBC<T>::Ainv_, JtBar);
    qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
    Npre = Npre * (_eye - JtBar * JtPre);
    // pretty_print(xddot, std::cout, "xddot");
    // pretty_print(JtDotQdot, std::cout, "JtDotQdot");
    // pretty_print(qddot_pre, std::cout, "qddot 2");
    // pretty_print(Jt, std::cout, "Jt");
    // pretty_print(JtPre, std::cout, "JtPre");
    // pretty_print(JtBar, std::cout, "JtBar");
  }
  task = (*_task_list)[0];
  task->getTaskJacobian(Jt);
  task->getTaskJacobianDotQdot(JtDotQdot);
  //task->getCommand(xddot);
  //cout<<"Addq+dAdq"<<(Jt*qddot_pre+JtDotQdot).transpose()<<endl;
  //cout<<"yei3"<<endl;
  _SetEqualityConstraint(qddot_pre);  // Set equality constraints
  //cout<<"yei4"<<endl;
  // printf("G:\n");
  // std::cout<<G<<std::endl;
  // printf("g0:\n");
  // std::cout<<g0<<std::endl;
  //Timer timer;
  T f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
  //std::cout<<"WBRC Time: "<<timer.getMs()<<std::endl;
  (void)f;
  //cout<<"yei5"<<endl;
  //cout<<"isFBcon: "<<isFBcon<<endl;
  // pretty_print(qddot_pre, std::cout, "qddot_cmd");
  if(isFBcon&&_dim_rf==0){//get rid of _dim_rf
    for (size_t i(0); i < _dim_floating+WBC<T>::num_act_joint_; ++i) qddot_pre[i] += z[i];
  }else{
    for (size_t i(0); i < _dim_floating; ++i) qddot_pre[i] += z[i];
  }
  //cout<<"yei6"<<endl;
  _GetSolution(qddot_pre, cmd);
  //cout<<"yei7"<<endl;
  wbc_data->_opt_result = DVec<T>(_dim_opt);
  //cout<<"yei8"<<endl;
  for (size_t i(0); i < _dim_opt; ++i) {
    wbc_data->_opt_result[i] = z[i];
  }
  //cout<<"yei9"<<endl;
  // std::cout << "f: " << f << std::endl;
  //std::cout << "x: " << z << std::endl;
  // DVec<T> check_eq = _dyn_CE * wbc_data->_opt_result + _dyn_ce0;
  // pretty_print(check_eq, std::cout, "equality constr");
  // std::cout << "cmd: "<<cmd<<std::endl;
  // pretty_print(qddot_pre, std::cout, "qddot_pre");
  // pretty_print(JcN, std::cout, "JcN");
  // pretty_print(Nci_, std::cout, "Nci");
  // DVec<T> eq_check = dyn_CE * data_->opt_result_;
  // pretty_print(dyn_ce0, std::cout, "dyn ce0");
  // pretty_print(eq_check, std::cout, "eq_check");

  // pretty_print(Jt, std::cout, "Jt");
  // pretty_print(JtDotQdot, std::cout, "Jtdotqdot");
  // pretty_print(xddot, std::cout, "xddot");

  // printf("CE:\n");
  // std::cout<<CE<<std::endl;
  // printf("ce0:\n");
  // std::cout<<ce0<<std::endl;

  // printf("CI:\n");
  // std::cout<<CI<<std::endl;
  // printf("ci0:\n");
  // std::cout<<ci0<<std::endl;
}

template <typename T>
void WBRC<T>::_SetEqualityConstraint(const DVec<T>& qddot) {
  if (_dim_rf > 0) {
    _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) = WBC<T>::A_.block(0, 0, _dim_floating, _dim_floating);
    //_dyn_CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) = -WBC<T>::Sv_ * _JcFBD.transpose();
    _dyn_CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) = -WBC<T>::Sv_ * _Jc.transpose();
    //_dyn_ce0 = -WBC<T>::Sv_ * (WBC<T>::A_ * qddot + WBC<T>::cori_ + WBC<T>::grav_ - _JcFBD.transpose() * _Fr_des);
    _dyn_ce0 = -WBC<T>::Sv_ * (WBC<T>::A_ * qddot + WBC<T>::cori_ + WBC<T>::grav_ - _Jc.transpose() * _Fr_des);
  } else {
    if(isFBcon){
      _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating+WBC<T>::num_act_joint_) = WBC<T>::A_.block(0, 0, _dim_floating, _dim_floating+WBC<T>::num_act_joint_);
    }else{
      _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) = WBC<T>::A_.block(0, 0, _dim_floating, _dim_floating);
    }
    _dyn_ce0 = -WBC<T>::Sv_ * (WBC<T>::A_ * qddot + WBC<T>::cori_ + WBC<T>::grav_);
  }

  for (size_t i(0); i < _dim_eq_cstr; ++i) {
    for (size_t j(0); j < _dim_opt; ++j) {
      CE[j][i] = _dyn_CE(i, j);
    }
    ce0[i] = -_dyn_ce0[i];
  }
  // pretty_print(_dyn_CE, std::cout, "WBRC: CE");
  // pretty_print(_dyn_ce0, std::cout, "WBRC: ce0");
}

template <typename T>
void WBRC<T>::_SetInEqualityConstraint() {
  _dyn_CI.block(0, _dim_floating, _dim_Uf, _dim_rf) = _Uf;
  _dyn_ci0 = _Uf_ieq_vec - _Uf * _Fr_des;
  for (size_t i(0); i < _dim_Uf; ++i) {
    for (size_t j(0); j < _dim_opt; ++j) {
      CI[j][i] = _dyn_CI(i, j);
    }
    ci0[i] = -_dyn_ci0[i];
  }
  // pretty_print(_dyn_CI, std::cout, "WBRC: CI");
  // pretty_print(_dyn_ci0, std::cout, "WBRC: ci0");
}

template <typename T>
void WBRC<T>::_ContactBuilding() {
  DMat<T> Uf, Jc;//, JcFBD;
  DVec<T> Uf_ieq_vec, JcDotQdot;
  size_t dim_accumul_rf, dim_accumul_uf;
  (*_contact_list)[0]->getContactJacobian(Jc);
  //(*_contact_list)[0]->getFBDJacobian(JcFBD);
  (*_contact_list)[0]->getJcDotQdot(JcDotQdot);
  (*_contact_list)[0]->getRFConstraintMtx(Uf);
  (*_contact_list)[0]->getRFConstraintVec(Uf_ieq_vec);
  dim_accumul_rf = (*_contact_list)[0]->getDim();
  dim_accumul_uf = (*_contact_list)[0]->getDimRFConstraint();
  _Jc.block(0, 0, dim_accumul_rf, WBC<T>::num_qdot_) = Jc;
  //_JcFBD.block(0, 0, dim_accumul_rf, WBC<T>::num_qdot_) = JcFBD;
  _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
  _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
  _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;
  _Fr_des.head(dim_accumul_rf) = (*_contact_list)[0]->getRFDesired();
  size_t dim_new_rf, dim_new_uf;
  for (size_t i(1); i < (*_contact_list).size(); ++i) {
    (*_contact_list)[i]->getContactJacobian(Jc);
    //(*_contact_list)[i]->getFBDJacobian(JcFBD);
    (*_contact_list)[i]->getJcDotQdot(JcDotQdot);
    dim_new_rf = (*_contact_list)[i]->getDim();
    dim_new_uf = (*_contact_list)[i]->getDimRFConstraint();
    _Jc.block(dim_accumul_rf, 0, dim_new_rf, WBC<T>::num_qdot_) = Jc;
    //_JcFBD.block(dim_accumul_rf, 0, dim_new_rf, WBC<T>::num_qdot_) = JcFBD;
    _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;
    (*_contact_list)[i]->getRFConstraintMtx(Uf);
    _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;
    (*_contact_list)[i]->getRFConstraintVec(Uf_ieq_vec);
    _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;
    _Fr_des.segment(dim_accumul_rf, dim_new_rf) = (*_contact_list)[i]->getRFDesired();
    dim_accumul_rf += dim_new_rf;
    dim_accumul_uf += dim_new_uf;
  }
  //pretty_print(_Fr_des, std::cout, "[WBRC] Fr des");
  //pretty_print(_Jc, std::cout, "[WBRC] Jc");
  //pretty_print(_JcFBD, std::cout, "[WBRC] JcFBC");
  // pretty_print(_JcDotQdot, std::cout, "[WBRC] JcDot Qdot");
  // pretty_print(_Uf, std::cout, "[WBRC] Uf");
}

template <typename T>
void WBRC<T>::_GetSolution(const DVec<T>& qddot, DVec<T>& cmd) {
  DVec<T> tot_tau;
  if (_dim_rf > 0) {
    wbc_data->_Fr = DVec<T>(_dim_rf);// get Reaction forces
    for (size_t i(0); i < _dim_rf; ++i)
      wbc_data->_Fr[i] = z[i + _dim_floating] + _Fr_des[i];
    //tot_tau = WBC<T>::A_ * qddot + WBC<T>::cori_ + WBC<T>::grav_ - _JcFBD.transpose() * wbc_data->_Fr;
    tot_tau = WBC<T>::A_ * qddot + WBC<T>::cori_ + WBC<T>::grav_ - _Jc.transpose() * wbc_data->_Fr;
    /*if(_dim_rf%3==0){
      cout<<wbc_data->_Fr[2]<<endl;
    }else{
      cout<<wbc_data->_Fr[0]<<endl;
    }*/
  } else {
    tot_tau = WBC<T>::A_ * qddot + WBC<T>::cori_ + WBC<T>::grav_;
  }
  wbc_data->_qddot = qddot;
  cmd = tot_tau.tail(WBC<T>::num_act_joint_);
  
  /*DVec<T> delta_tau = DVec<T>::Zero(_dim_floating);
  for(size_t i(0); i<_dim_floating; ++i) delta_tau[i] = z[i];
  cout<<"delta_tau :"<<delta_tau.transpose()<<endl;*/
  //cout<<"A_ :"<<WBC<T>::A_<<endl;
  //pretty_print(delta_tau, std::cout, "delta_tau");
  // Torque check
  /*DVec<T> delta_tau = DVec<T>::Zero(WBC<T>::num_qdot_);
  for(size_t i(0); i<_dim_floating; ++i) delta_tau[i] = z[i];
  pretty_print(tot_tau, std::cout, "tot tau original");
  tot_tau += delta_tau;
  pretty_print(tot_tau, std::cout, "tot tau result");
  pretty_print(delta_tau, std::cout, "delta_tau");*/
  //pretty_print(qddot, std::cout, "qddot");
  //pretty_print(wbc_data->_Fr, std::cout, "Fr");
  //pretty_print(_Fr_des, std::cout, "Fr des");
}

template <typename T>
void WBRC<T>::_SetCost() {// Set Cost
  if(isFBcon&&_dim_rf==0){//get rid of _dim_rf
    for (size_t i(0); i < 3; ++i) {
      G[i][i] = 30.f;//200
    }
    G[1][1] = 1.f;//10
    for (size_t i(3); i < 6; ++i) {
      G[i][i] = .001f;
    }
    for(size_t leg(0); leg<4; ++leg){
      G[3*leg + 0 + _dim_floating][3*leg + 0 + _dim_floating] = .4f;
      G[3*leg + 1 + _dim_floating][3*leg + 1 + _dim_floating] = 1.f;
      G[3*leg + 2 + _dim_floating][3*leg + 2 + _dim_floating] = 1.f;
    }
    /*for (size_t i(0); i < WBC<T>::num_act_joint_; ++i) {
      G[i + _dim_floating][i + _dim_floating] = wbc_data->_W_floating[0];//Improve
    }*/
  }else{
    size_t idx_offset(0);
    for (size_t i(0); i < _dim_floating; ++i) {
      G[i + idx_offset][i + idx_offset] = wbc_data->_W_floating[i];
    }
    idx_offset += _dim_floating;
    for (size_t i(0); i < _dim_rf; ++i) {
      G[i + idx_offset][i + idx_offset] = wbc_data->_W_rf[i];
    }
  }
  // pretty_print(wbc_data->_W_floating, std::cout, "W floating");
  // pretty_print(wbc_data->_W_rf, std::cout, "W rf");
}

template <typename T>
void WBRC<T>::UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
    const DVec<T>& cori, const DVec<T>& grav,
    void* extra_setting) {
  WBC<T>::A_ = A;
  WBC<T>::Ainv_ = Ainv;
  WBC<T>::cori_ = cori;
  WBC<T>::grav_ = grav;
  WBC<T>::b_updatesetting_ = true;
  (void)extra_setting;
}

template <typename T>
void WBRC<T>::_SetOptimizationSize() {
  _dim_rf = 0;  // Dimension
  _dim_Uf = 0;  // Dimension of inequality constraint
  for (size_t i(0); i < (*_contact_list).size(); ++i) {
    _dim_rf += (*_contact_list)[i]->getDim();
    _dim_Uf += (*_contact_list)[i]->getDimRFConstraint();
  }
  _dim_opt = _dim_floating + _dim_rf;
  _dim_eq_cstr = _dim_floating;
  G.resize(0., _dim_opt, _dim_opt);// Matrix Setting
  g0.resize(0., _dim_opt);
  CE.resize(0., _dim_opt, _dim_eq_cstr);
  ce0.resize(0., _dim_eq_cstr);
  _dyn_CE = DMat<T>::Zero(_dim_eq_cstr, _dim_opt);// Eigen Matrix Setting
  _dyn_ce0 = DVec<T>(_dim_eq_cstr);
  if (_dim_rf > 0) {
    CI.resize(0., _dim_opt, _dim_Uf);
    ci0.resize(0., _dim_Uf);
    _dyn_CI = DMat<T>::Zero(_dim_Uf, _dim_opt);
    _dyn_ci0 = DVec<T>(_dim_Uf);
    _Jc = DMat<T>(_dim_rf, WBC<T>::num_qdot_);//this->num_qdot_
    //_JcFBD = DMat<T>(_dim_rf, WBC<T>::num_qdot_);
    _JcDotQdot = DVec<T>(_dim_rf);
    _Fr_des = DVec<T>(_dim_rf);
    _Uf = DMat<T>(_dim_Uf, _dim_rf);
    _Uf.setZero();
    _Uf_ieq_vec = DVec<T>(_dim_Uf);
  } else {
    if(isFBcon){
      _dim_opt = WBC<T>::num_qdot_;//_dim_floating+
      G.resize(0., _dim_opt, _dim_opt);// Matrix Setting
      g0.resize(0., _dim_opt);
      CE.resize(0., _dim_opt, _dim_eq_cstr);
      _dyn_CE = DMat<T>::Zero(_dim_eq_cstr, _dim_opt);// Eigen Matrix Setting
    }
    CI.resize(0., _dim_opt, 1);
    ci0.resize(0., 1);
  }
}

template class WBRC<double>;
template class WBRC<float>;