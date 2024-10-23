#include "CentroidAngMomentumTask2.hpp"

#include <Configuration.h>
#include <FBModel/FloatingBaseModel.h>
#include <orientation_tools.h>
#include <pretty_print.h>


template <typename T>
CentroidAngMomentumTask2<T>::CentroidAngMomentumTask2(const FloatingBaseModel<T>* robot)
    : Task<T>(3), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, robot->_nDof);
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 0.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 0.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 80);

}

template <typename T>
CentroidAngMomentumTask2<T>::~CentroidAngMomentumTask2() {}

template <typename T>
bool CentroidAngMomentumTask2<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {
  // vel_des is the desired centroidal momentem (angular component only)
  
  Quat<T>* ori_cmd = (Quat<T>*)pos_des;
  Quat<T> link_ori = (_robot_sys->_state.bodyOrientation);

  Quat<T> link_ori_inv;
  link_ori_inv[0] = link_ori[0];
  link_ori_inv[1] = -link_ori[1];
  link_ori_inv[2] = -link_ori[2];
  link_ori_inv[3] = -link_ori[3];

  // Explicit because operational space is in global frame
  Quat<T> ori_err = ori::quatProduct(*ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) {
    ori_err *= (-1.);
  }
  Vec3<T> ori_err_so3;
  ori::quaternionToso3(ori_err, ori_err_so3);

  // Centroidal Velocity
  Vec6<T> hG = _robot_sys->getCompositeRBI() * _robot_sys->getCentroidalVelocity();

  Vec3<T> hGdot_cmd;
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = ori_err_so3[i];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
    // vGdot_cmd[i] = acc_des[i] + _Kp[i]*TK::pos_err_[i] + _Kd[i] * (vel_des[i] - vG[i]);
    hGdot_cmd[i] = acc_des[i] + _Kd[i] * (vel_des[i] - hG[i]);
  }

  // full command
  TK::op_cmd_ = hGdot_cmd;
  
  //printf("[Centroid Angular Momentum Task]\n");
  // pretty_print(v, std::cout, "Spatial velocity (fb + joints)");
  // pretty_print(hg, std::cout, "Centroid Momentum");
  // pretty_print(vel_des, std::cout, "Desired momentum");
  // pretty_print(TK::op_cmd_, std::cout, "CAM command");

  // std::cout << "[CAM Task] Updated Command" << std::endl;
  return true;
}

template <typename T>
bool CentroidAngMomentumTask2<T>::_UpdateTaskJacobian() {

  // Task Jacobian
  TK::Jt_ = (_robot_sys->getCMM()).block(0,0,TK::dim_task_, _robot_sys->_nDof);

  // pretty_print(TK::Jt_, std::cout, "Jt CAM");
  // std::cout << "[CAM Task] Updated Task Jacobian" << std::endl;
  return true;
}

template <typename T>
bool CentroidAngMomentumTask2<T>::_UpdateTaskJDotQdot() {

  TK::JtDotQdot_ = (_robot_sys->getCmmBiasForce()).head(3);

  // std::cout << "[CAM Task] Updated Task JDotQdot" << std::endl;
  //pretty_print(TK::JtDotQdot_, std::cout, "JtDotQdot CAM");
  return true;
}

template class CentroidAngMomentumTask2<double>;
template class CentroidAngMomentumTask2<float>;
