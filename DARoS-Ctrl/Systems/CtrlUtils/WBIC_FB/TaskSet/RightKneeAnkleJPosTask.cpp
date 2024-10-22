#include "RightKneeAnkleJPosTask.hpp"
#include <Configuration.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Utilities/pretty_print.h>

template <typename T>
RightKneeAnkleJPosTask<T>::RightKneeAnkleJPosTask(const FloatingBaseModel<T>* robot)
    : Task<T>(2), robot_sys_(robot) {
   // nb joints for this jpos task.

  TK::Jt_ = DMat<T>::Zero(Nj, robot->_nDof);
  (TK::Jt_.block(0, 9, Nj, Nj)).setIdentity();
  TK::JtDotQdot_ = DVec<T>::Zero(Nj);

  _Kp = DVec<T>::Constant(Nj, 50.);
  _Kd = DVec<T>::Constant(Nj, 5.);
}

template <typename T>
RightKneeAnkleJPosTask<T>::~RightKneeAnkleJPosTask() {}

template <typename T>
bool RightKneeAnkleJPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                 const DVec<T>& acc_des) {
  Vec2<T>* pos_cmd = (Vec2<T>*)pos_des;

  for (size_t i(0); i < Nj; ++i) {
    TK::pos_err_[i] = (*pos_cmd)[i] - robot_sys_->_state.q[i + 3];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
    TK::op_cmd_[i] = _Kp[i] * TK::pos_err_[i] +
                     _Kd[i] * (vel_des[i] - robot_sys_->_state.qd[i + 3]) +
                     acc_des[i];
  }
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(op_cmd_, std::cout, "op cmd");
  // pretty_print(*pos_cmd, std::cout, "pos cmd");

  return true;
}

template <typename T>
bool RightKneeAnkleJPosTask<T>::_UpdateTaskJacobian() {
  return true;
}

template <typename T>
bool RightKneeAnkleJPosTask<T>::_UpdateTaskJDotQdot() {
  return true;
}

template class RightKneeAnkleJPosTask<double>;
template class RightKneeAnkleJPosTask<float>;
