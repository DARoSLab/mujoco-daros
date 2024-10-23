#include "CentroidMomentumTask.hpp"

#include <Configuration.h>
#include <FBModel/FloatingBaseModel.h>
#include <orientation_tools.h>
#include <pretty_print.h>


template <typename T>
CentroidMomentumTask<T>::CentroidMomentumTask(const FloatingBaseModel<T>* robot)
    : Task<T>(6), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, robot->_nDof);
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 0.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 0.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 80);

}

template <typename T>
CentroidMomentumTask<T>::~CentroidMomentumTask() {}

template <typename T>
bool CentroidMomentumTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {
  // pos des = [quat_des;com_des]
  // vel des = vG_des = [angular;linear] (Centroidal velocity)
  // acc des = aG_des = (Centroidal acceleration)

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

  Quat<T> ori_err = ori::quatProduct(ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) {
    ori_err *= (-1.);
  }
  Vec3<T> ori_err_so3;
  ori::quaternionToso3(ori_err, ori_err_so3);

  // Centroidal Velocity
  Vec6<T> vG = _robot_sys->getCentroidalVelocity();

  // Rx, Ry, Rz
  // TODO: add angular excursion for absolute orientation tracking
  Vec6<T> vGdot_cmd;
  for (int i(0); i < 3; ++i) {
    //TK::pos_err_[i] = ori_err_so3[i];
    TK::pos_err_[i] = 0.;// ori_err_so3[i];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
    vGdot_cmd[i] = acc_des[i] + _Kp[i]*TK::pos_err_[i] + _Kd[i] * (vel_des[i] - vG[i]);
  }

  // Position
  Vec3<T> com_pos = _robot_sys->_state.bodyPosition + _robot_sys->getComPosWorld();
  // X, Y, Z
  for (int i(3); i < 6; ++i) {
    TK::pos_err_[i] = (*pos_cmd)[i + 1] - com_pos[i - 3];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
    vGdot_cmd[i] = _Kp[i] * TK::pos_err_[i] +
                    _Kd[i] * (TK::vel_des_[i] - vG[i]) + 
                    TK::acc_des_[i];

   }

  // full command
  TK::op_cmd_ = vGdot_cmd;

   //pretty_print(h, std::cout, "Centroid Momentum");
   //Mat6<T> Ig = _robot_sys->getCompositeRBI();
   //pretty_print(Ig, std::cout, "Ig");
    //pretty_print(*pos_cmd, std::cout, "pos cmd");
    //pretty_print(com_pos, std::cout, "CoM pos");
  // pretty_print(hdot_cmd, std::cout, "hdot cmd");
   //pretty_print(TK::op_cmd_, std::cout, "op cmd");
  // std::cout << "[CM Task] Updated Command" << std::endl;
  return true;
}

template <typename T>
bool CentroidMomentumTask<T>::_UpdateTaskJacobian() {

  TK::Jt_ = (_robot_sys->getCompositeRBI()).inverse() * _robot_sys->getCMM();

  // pretty_print(_robot_sys->getCMM(), std::cout, "Centroid Momentum Matrix");
  // pretty_print(_robot_sys->getCompositeRBI(), std::cout, "Ig");
  // pretty_print(TK::Jt_, std::cout, "Jt CM");
  // std::cout << "[CM Task] Updated Task Jacobian" << std::endl;
  return true;
}

template <typename T>
bool CentroidMomentumTask<T>::_UpdateTaskJDotQdot() {

  // Mat6<T> Identity, Ig_inv, Igd;
  // Identity.setIdentity();
  // Ig_inv = (_robot_sys->getCompositeRBI()).inverse();
  // Igd = _robot_sys->getCompositeRBIdot();
  // TK::JtDotQdot_ = Ig_inv*(_robot_sys->getCmmBiasForce() - Igd*_robot_sys->getCentroidalVelocity());

  // Old
  //TK::JtDotQdot_ = (_robot_sys->getCompositeRBI()).inverse() * _robot_sys->getCmmBiasForce(); 
  TK::JtDotQdot_.setZero();
  // Incorrect! See Charles note's on computing _vdotG

  // pretty_print(TK::JtDotQdot_, std::cout, "JtDotQdot");
  // pretty_print(_robot_sys->getCmmBiasForce(), std::cout, "CMM Bias Force");
  // std::cout << "[CM Task] Updated Task JDotQdot" << std::endl;
  return true;
}

template class CentroidMomentumTask<double>;
template class CentroidMomentumTask<float>;
