#include "JPosTask.hpp"
#include <Configuration.h>
#include <FBModel/FloatingBaseModel.h>
#include <pretty_print.h>

template <typename T>
JPosTask<T>::JPosTask(const FloatingBaseModel<T>* robot)
    : Task<T>(robot->_nDof-6), robot_sys_(robot) {
  TK::Jt_ = DMat<T>::Zero(robot->_nDof-6, robot->_nDof);
  (TK::Jt_.block(0, 6, robot->_nDof-6, robot->_nDof-6))
      .setIdentity();
  TK::JtDotQdot_ = DVec<T>::Zero(robot->_nDof-6);

  _Kp = DVec<T>::Constant(robot->_nDof-6, 50.);
  _Kd = DVec<T>::Constant(robot->_nDof-6, 5.);
}

template <typename T>
JPosTask<T>::~JPosTask() {}

template <typename T>
bool JPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                 const DVec<T>& acc_des) {
  DVec<T>* pos_cmd = (DVec<T>*)pos_des;

  for (size_t i(0); i < robot_sys_->_nDof-6; ++i) {
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
bool JPosTask<T>::_UpdateTaskJacobian() {
  return true;
}

template <typename T>
bool JPosTask<T>::_UpdateTaskJDotQdot() {
  return true;
}

template class JPosTask<double>;
template class JPosTask<float>;
