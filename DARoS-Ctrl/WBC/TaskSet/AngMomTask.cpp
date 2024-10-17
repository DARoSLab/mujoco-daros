#include "AngMomTask.hpp"
#include <Configuration.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Utilities/pretty_print.h>

template <typename T>
AngMomTask<T>::AngMomTask(const FloatingBaseModel<T>* robot) : Task<T>(3), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, robot->_nDof);
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);
  _Kp = DVec<T>::Constant(TK::dim_task_, 10);
}

template <typename T>
AngMomTask<T>::~AngMomTask() {}

template <typename T>
bool AngMomTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& L_des, const DVec<T>& Ld_des){
  //SVec<T> hG = (_robot_sys->getCompositeRBI()*_robot_sys->getCentroidalVelocity());
  //DVec<T>* pos_cmd = (DVec<T>*)pos_des;
  Quat<T>* ori_cmd = (Quat<T>*)pos_des;
  Quat<T> link_ori = (_robot_sys->_state.bodyOrientation);
  Quat<T> link_ori_inv;
  link_ori_inv[0] = link_ori[0];
  link_ori_inv[1] = -link_ori[1];
  link_ori_inv[2] = -link_ori[2];
  link_ori_inv[3] = -link_ori[3];
  Quat<T> ori_err = ori::quatProduct(*ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) ori_err *= (-1.);
  Vec3<T> ori_err_so3;
  ori::quaternionToso3(ori_err, ori_err_so3);
  
  Vec3<T> L_curr=(_robot_sys->getCompositeRBI()*_robot_sys->getCentroidalVelocity()).head(3);// = _robot_sys->_state.bodyVelocity;
  Vec3<T> L_err = L_des-L_curr;
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = ori_err_so3[i];
    TK::vel_des_[i] = L_des[i];
    TK::acc_des_[i] = Ld_des[i];
    TK::op_cmd_[i] = _Kp[i] * L_err[i] + TK::acc_des_[i];
  }
  /*printf("[Ang Mom Task]\n");
  std::cout<<"L_des \t"<<L_des.transpose()<<"L_curr \t"<<L_curr.transpose()<<std::endl;
  pretty_print(L_des, std::cout, "L_des");
  pretty_print(L_curr, std::cout, "L_curr");
  pretty_print(Ld_des, std::cout, "Ld_des");
  pretty_print(TK::Jt_, std::cout, "Jt");*/
  return true;
}

template <typename T>
bool AngMomTask<T>::_UpdateTaskJacobian() {
  TK::Jt_ = _robot_sys->getCMM().block(0,0,TK::dim_task_, _robot_sys->_nDof);
  return true;
}

template <typename T>
bool AngMomTask<T>::_UpdateTaskJDotQdot() {
  TK::JtDotQdot_ = _robot_sys->getCmmBiasForce().head(3);
  return true;
}

template class AngMomTask<double>;
template class AngMomTask<float>;