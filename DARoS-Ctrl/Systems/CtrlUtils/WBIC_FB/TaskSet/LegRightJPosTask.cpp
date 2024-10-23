#include "LegRightJPosTask.hpp"
#include <Configuration.h>
#include <FBModel/FloatingBaseModel.h>
#include <pretty_print.h>

template <typename T>
LegRightJPosTask<T>::LegRightJPosTask(const FloatingBaseModel<T>* robot)
    : Task<T>(5), robot_sys_(robot) {
  TK::Jt_ = DMat<T>::Zero(5, robot->_nDof);
  (TK::Jt_.block(0, 6, 5, 5)).setIdentity();
  TK::JtDotQdot_ = DVec<T>::Zero(5);

  _Kp = DVec<T>::Constant(5, 50.);
  _Kd = DVec<T>::Constant(5, 5.);
}

template <typename T>
LegRightJPosTask<T>::~LegRightJPosTask() {}

template <typename T>
bool LegRightJPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                 const DVec<T>& acc_des) {
  Vec5<T>* pos_cmd = (Vec5<T>*)pos_des;

  for (size_t i(0); i < 5; ++i) {
    TK::pos_err_[i] = (*pos_cmd)[i] - robot_sys_->_state.q[i];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    TK::op_cmd_[i] = _Kp[i] * TK::pos_err_[i] +
                     _Kd[i] * (vel_des[i] - robot_sys_->_state.qd[i]) +
                     acc_des[i];
  }
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(op_cmd_, std::cout, "op cmd");
  // pretty_print(*pos_cmd, std::cout, "pos cmd");

  return true;
}

template <typename T>
bool LegRightJPosTask<T>::_UpdateTaskJacobian() {
  return true;
}

template <typename T>
bool LegRightJPosTask<T>::_UpdateTaskJDotQdot() {
  return true;
}

template class LegRightJPosTask<double>;
template class LegRightJPosTask<float>;
