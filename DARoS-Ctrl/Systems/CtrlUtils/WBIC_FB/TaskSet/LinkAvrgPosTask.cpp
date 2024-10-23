#include "LinkAvrgPosTask.hpp"
#include <Configuration.h>
#include <FBModel/FloatingBaseModel.h>
#include <pretty_print.h>

template <typename T>
LinkAvrgPosTask<T>::LinkAvrgPosTask(const FloatingBaseModel<T>* robot, int link_idx1, int link_idx2, bool virtual_depend)
    : Task<T>(3),
      robot_sys_(robot),
      link_idx1_(link_idx1),
      link_idx2_(link_idx2),
      virtual_depend_(virtual_depend) {
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);
  _Kp = DVec<T>::Constant(TK::dim_task_, 500.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 20.);
  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);
}

template <typename T>
LinkAvrgPosTask<T>::~LinkAvrgPosTask() {}

template <typename T>
bool LinkAvrgPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des) {
  Vec3<T>* pos_cmd = (Vec3<T>*)pos_des;
  Vec3<T> link_pos;
  link_pos = .5f*(robot_sys_->_pGC[link_idx1_]+robot_sys_->_pGC[link_idx2_]);
  for (size_t i(0); i < TK::dim_task_; ++i) {
    TK::pos_err_[i] = _Kp_kin[i]* ( (*pos_cmd)[i] - link_pos[i] );
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
    TK::op_cmd_[i] = _Kp[i] * TK::pos_err_[i] + _Kd[i] * (TK::vel_des_[i] - .5f*(robot_sys_->_vGC[link_idx1_][i]+robot_sys_->_vGC[link_idx2_][i])) + TK::acc_des_[i];
  }
   //printf("[Link Pos Task]\n");
   //pretty_print(acc_des, std::cout, "acc_des");
   //pretty_print(vel_des, std::cout, "vel des");
   //pretty_print(TK::pos_err_, std::cout, "pos_err_");
   //pretty_print(*pos_cmd, std::cout, "pos cmd");
   //pretty_print(robot_sys_->_vGC[link_idx1_], std::cout, "velocity1");
   //pretty_print(robot_sys_->_vGC[link_idx2_], std::cout, "velocity2");
   //pretty_print(.5f*(robot_sys_->_vGC[link_idx1_]+robot_sys_->_vGC[link_idx2_]), std::cout, "velocityavrg");
   //pretty_print(TK::op_cmd_, std::cout, "op cmd");
   //pretty_print(TK::Jt_, std::cout, "Jt");
  return true;
}

template <typename T>
bool LinkAvrgPosTask<T>::_UpdateTaskJacobian() {
  TK::Jt_ = .5f*(robot_sys_->_Jc[link_idx1_]+robot_sys_->_Jc[link_idx2_]);
  if (!virtual_depend_) {
    TK::Jt_.block(0, 0, 3, 6) = DMat<T>::Zero(3, 6);
  }
  return true;
}

template <typename T>
bool LinkAvrgPosTask<T>::_UpdateTaskJDotQdot() {
  TK::JtDotQdot_ = .5f*(robot_sys_->_Jcdqd[link_idx1_]+robot_sys_->_Jcdqd[link_idx2_]);
  return true;
}

template class LinkAvrgPosTask<double>;
template class LinkAvrgPosTask<float>;