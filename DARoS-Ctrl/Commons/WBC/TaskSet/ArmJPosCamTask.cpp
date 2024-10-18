#include "ArmJPosCamTask.hpp"

#include <Configuration.h>
#include <Dynamics/FloatingBaseModel.h>
#include <math/orientation_tools.h>
#include <Utilities/pretty_print.h>


template <typename T>
ArmJPosCamTask<T>::ArmJPosCamTask(const FloatingBaseModel<T>* robot)
    : Task<T>(6+3), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, robot->_nDof);
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 0.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 100.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 20.);

}

template <typename T>
ArmJPosCamTask<T>::~ArmJPosCamTask() {}

template <typename T>
bool ArmJPosCamTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {
  // pos_des = [arm_joint_positions;body quaternion]
  // vel_des = [arm_joint_vel;average angular velocity]
  // acc_des = [arm_joint_accel;average angular acceleration]

  DVec<T>* pos_cmd = (DVec<T>*)pos_des;

  // Arm Joint Pos Task
  for (size_t i(0); i < 6; ++i) { // ideally wil remove the hard coded 6
    TK::pos_err_[i] = (*pos_cmd)[i] - _robot_sys->_state.q[16+i];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    TK::op_cmd_[i] = _Kp[i] * TK::pos_err_[i] +
                     _Kd[i] * (vel_des[i] - _robot_sys->_state.qd[16+i]) +
                     acc_des[i];
  }

  // CAM Task
  Quat<T> ori_cmd;
  for(size_t i(0); i < 4; i++){
    ori_cmd[i] = (*pos_cmd)[6+i];
  }
  Quat<T> link_ori = (_robot_sys->_state.bodyOrientation);

  Quat<T> link_ori_inv;
  link_ori_inv[0] = link_ori[0];
  link_ori_inv[1] = -link_ori[1];
  link_ori_inv[2] = -link_ori[2];
  link_ori_inv[3] = -link_ori[3];

  // Explicit because operational space is in global frame
  Quat<T> ori_err = ori::quatProduct(ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) {
    ori_err *= (-1.);
  }
  Vec3<T> ori_err_so3;
  ori::quaternionToso3(ori_err, ori_err_so3);

  // Centroidal Velocity
  Vec6<T> vG = _robot_sys->getCentroidalVelocity();

  for (int i(0); i < 3; ++i) {
    TK::pos_err_[6+i] = ori_err_so3[i];
    TK::vel_des_[6+i] = vel_des[6+i];
    TK::acc_des_[6+i] = acc_des[6+i];
    TK::op_cmd_[6+i] = acc_des[6+i] + _Kp[6+i]*TK::pos_err_[6+i] + _Kd[6+i] * (vel_des[6+i] - vG[i]);
  }
  
  return true;
}

template <typename T>
bool ArmJPosCamTask<T>::_UpdateTaskJacobian() {

  // Task Jacobian
  (TK::Jt_.block(0, 16, 6, 6)).setIdentity();
  TK::Jt_.block(6, 0, 3, _robot_sys->_nDof) = ((_robot_sys->getCompositeRBI()).inverse() * _robot_sys->getCMM()).block(0,0,3, _robot_sys->_nDof);

  // pretty_print(TK::Jt_, std::cout, "Jt Arm Pos/CAM");
  // std::cout << "[Arm Pos/CAM Task] Updated Task Jacobian" << std::endl;
  return true;
}

template <typename T>
bool ArmJPosCamTask<T>::_UpdateTaskJDotQdot() {

  TK::JtDotQdot_.head(6) = DVec<T>::Zero(6);
  TK::JtDotQdot_.tail(3) = ((_robot_sys->getCompositeRBI()).inverse() * _robot_sys->getCmmBiasForce()).head(3);

  return true;
}

template class ArmJPosCamTask<double>;
template class ArmJPosCamTask<float>;
