#include "JPosBodyOriTask.hpp"
// (Rx, Ry, Rz, jpos)

#include <Configuration.h>
#include <FBModel/FloatingBaseModel.h>
#include <orientation_tools.h>
#include <pretty_print.h>


template <typename T>
JPosBodyOriTask<T>::JPosBodyOriTask(const FloatingBaseModel<T>* robot)
    : Task<T>(robot->_nDof - 3), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, robot->_nDof);
  TK::Jt_.block(0, 0, 3, 3).setIdentity();
  TK::Jt_.block(3, 6, robot->_nDof-6, robot->_nDof-6).setIdentity();
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 50.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 1.);
}

template <typename T>
JPosBodyOriTask<T>::~JPosBodyOriTask() {}

template <typename T>
bool JPosBodyOriTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {
  DVec<T>* pos_cmd = (DVec<T>*)pos_des;

  // Orientation (w, x, y, z)
  Quat<T> ori_cmd;
  for (size_t i(0); i < 4; ++i) ori_cmd[i] = (*pos_cmd)[i];
  Quat<T> link_ori = (_robot_sys->_state.bodyOrientation);
  Quat<T> link_ori_inv;
  link_ori_inv[0] = link_ori[0];
  link_ori_inv[1] = -link_ori[1];
  link_ori_inv[2] = -link_ori[2];
  link_ori_inv[3] = -link_ori[3];
  //link_ori_inv /= link_ori.norm();

  // Explicit because operational space is in global frame
  Quat<T> ori_err = ori::quatProduct(ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) {
    ori_err *= (-1.);
  }
  Vec3<T> ori_err_so3;
  ori::quaternionToso3(ori_err, ori_err_so3);
  SVec<T> curr_vel = _robot_sys->_state.bodyVelocity;

  // Configuration space: Local
  // Operational Space: Global
  Mat3<T> Rot = ori::quaternionToRotationMatrix(link_ori);
  Vec3<T> vel_err = Rot.transpose()*(vel_des.head(3) - curr_vel.head(3));

  // Rx, Ry, Rz
  for (size_t i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i] * ori_err_so3[i];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    TK::op_cmd_[i] = _Kp[i] * ori_err_so3[i] +
                     _Kd[i] * vel_err[i] + TK::acc_des_[i];
  }

  // Jpos
  for (size_t i(3); i < TK::dim_task_; ++i) {
    TK::pos_err_[i] = (*pos_cmd)[i + 1] - _robot_sys->_state.q[i-3];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    TK::op_cmd_[i] = _Kp[i] * TK::pos_err_[i] +
                     _Kd[i] * (TK::vel_des_[i] - _robot_sys->_state.qd[i-3]) + 
                     TK::acc_des_[i];
   }
   //printf("[JPos + Body Ori Task]\n");
   //pretty_print(TK::pos_err_, std::cout, "pos_err_");
   //pretty_print(*ori_cmd, std::cout, "des_ori");
   //pretty_print(link_ori, std::cout, "curr_ori");
   //pretty_print(ori_err, std::cout, "quat_err");

  // pretty_print(link_ori_inv, std::cout, "ori_inv");
  // pretty_print(ori_err, std::cout, "ori_err");
  // pretty_print(*ori_cmd, std::cout, "cmd");
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(TK::Jt_, std::cout, "Jt");

  return true;
}

template <typename T>
bool JPosBodyOriTask<T>::_UpdateTaskJacobian() {
  Quat<T> quat = _robot_sys->_state.bodyOrientation;
  Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
  TK::Jt_.block(0, 0, 3, 3) = Rot.transpose();
  //pretty_print(Rot, std::cout, "Rot mat");
  //pretty_print(TK::Jt_, std::cout, "Jt ori");
  return true;
}

template <typename T>
bool JPosBodyOriTask<T>::_UpdateTaskJDotQdot() {
  return true;
}

template class JPosBodyOriTask<double>;
template class JPosBodyOriTask<float>;
