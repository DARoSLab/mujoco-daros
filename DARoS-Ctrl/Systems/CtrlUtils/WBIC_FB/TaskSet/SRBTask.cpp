#include "SRBTask.hpp"

#include <Configuration.h>
#include <FBModel/FloatingBaseModel.h>
#include <orientation_tools.h>
#include <pretty_print.h>


template <typename T>
SRBTask<T>::SRBTask(const FloatingBaseModel<T>* robot)
    : Task<T>(6), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, robot->_nDof);
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 0.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 0.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 80);

}

template <typename T>
SRBTask<T>::~SRBTask() {}

template <typename T>
bool SRBTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {
  // pos des = [quat_des;com_des]
  // vel des = vG_des = [angular;linear] (Centroidal velocity)
  // acc des = aG_des = (Centroidal acceleration)
  Vec6<T> vGdot_cmd;
  //orientation
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = 0.;
    TK::vel_des_[i] = 0.0;
    TK::acc_des_[i] = acc_des[i];
    vGdot_cmd[i] = acc_des[i];
  }
  //position
  for (int i(3); i < 6; ++i) {
    TK::pos_err_[i] = 0.0;
    TK::vel_des_[i] = 0.0;
    TK::acc_des_[i] = acc_des[i];
    vGdot_cmd[i] = TK::acc_des_[i];
   }
  // full command
  TK::op_cmd_ = vGdot_cmd;

  return true;
}

template <typename T>
bool SRBTask<T>::_UpdateTaskJacobian() {

  TK::Jt_ = (_robot_sys->getCompositeRBI()).inverse() * _robot_sys->getCMM();
  return true;
}

template <typename T>
bool SRBTask<T>::_UpdateTaskJDotQdot() {
  TK::JtDotQdot_.setZero();
  return true;
}

template class SRBTask<double>;
template class SRBTask<float>;
