#include "MomentumTask.hpp"
#include <Configuration.h>
#include <Dynamics/FloatingBaseModel.h>
#include <math/orientation_tools.h>
#include <Utilities/pretty_print.h>

template <typename T>
MomentumTask<T>::MomentumTask(const FloatingBaseModel<T>* robot) : Task<T>(6), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, robot->_nDof);
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);
  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 50.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 50.);
}

template <typename T>
MomentumTask<T>::~MomentumTask() {}

template <typename T>
bool MomentumTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des) { //vel_des=L/23.172f*_input_data->vBody_des
  DVec<T>* pos_cmd = (DVec<T>*)pos_des;
  Quat<T> ori_cmd;
  for (size_t i(0); i < 4; ++i) ori_cmd[i] = (*pos_cmd)[i];
  Quat<T> link_ori = (_robot_sys->_state.bodyOrientation);
  Quat<T> link_ori_inv;
  link_ori_inv[0] = link_ori[0];
  link_ori_inv[1] = -link_ori[1];
  link_ori_inv[2] = -link_ori[2];
  link_ori_inv[3] = -link_ori[3];

  Quat<T> ori_err = ori::quatProduct(ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) ori_err *= (-1.);
  Vec3<T> ori_err_so3;
  ori::quaternionToso3(ori_err, ori_err_so3);
  Vec6<T> vG = _robot_sys->getCentroidalVelocity();
  Vec6<T> hG = (_robot_sys->getCompositeRBI())*_robot_sys->getCentroidalVelocity();

  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = ori_err_so3[i];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
  }

  Vec3<T> com_pos = _robot_sys->_state.bodyPosition + _robot_sys->getComPosWorld();
  for (int i(3); i < 6; ++i) {
    TK::pos_err_[i] = (*pos_cmd)[i + 1] - com_pos[i - 3];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
  }

  Vec6<T> vG_des = (_robot_sys->getCompositeRBI().inverse())*TK::vel_des_;

  for (int i(0); i < 3; ++i) {
    TK::op_cmd_[i] = TK::acc_des_[i] + _Kp[i] * TK::pos_err_[i] + _Kd[i] * (vG_des[i] - vG[i]);
  }
  for (int i(3); i < 6; ++i) {
    TK::op_cmd_[i] = TK::acc_des_[i] + _Kp[i] * TK::pos_err_[i] + _Kd[i] * (TK::vel_des_[i] - hG[i])/23.172f;
  }

  /*std::cout<<"hG : "<<hG.transpose()<<"\th: "<<TK::vel_des_.transpose()<<"\tp_e: "<<TK::pos_err_.transpose()<<std::endl;
  std::cout<<"op_cmd_: "<<TK::op_cmd_.transpose()<<"\thd: "<<TK::acc_des_.transpose()<<std::endl;*/
  // pretty_print(com_pos, std::cout, "CoM pos");
  // pretty_print(hdot_cmd, std::cout, "hdot cmd");
  // pretty_print(TK::op_cmd_, std::cout, "op cmd");
  // std::cout << "[CM Task] Updated Command" << std::endl;
  return true;
}

template <typename T>
bool MomentumTask<T>::_UpdateTaskJacobian() {
  TK::Jt_ = _robot_sys->getCMM();
  // pretty_print(_robot_sys->getCMM(), std::cout, "Centroid Momentum Matrix");
  // pretty_print(_robot_sys->getCompositeRBI(), std::cout, "Ig");
  // pretty_print(TK::Jt_, std::cout, "Jt CM");
  // std::cout << "[CM Task] Updated Task Jacobian" << std::endl;
  return true;
}

template <typename T>
bool MomentumTask<T>::_UpdateTaskJDotQdot() {
  TK::JtDotQdot_ = _robot_sys->getCmmBiasForce(); // Incorrect! See Charles note's on computing _vdotG
  // pretty_print(TK::JtDotQdot_, std::cout, "JtDotQdot");
  return true;
}

template class MomentumTask<double>;
template class MomentumTask<float>;
