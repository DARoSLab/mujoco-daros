#include "CentroidLinearMomentumTask.hpp"

#include <Configuration.h>
#include <Dynamics/FloatingBaseModel.h>
#include <math/orientation_tools.h>
#include <Utilities/pretty_print.h>


template <typename T>
CentroidLinearMomentumTask<T>::CentroidLinearMomentumTask(const FloatingBaseModel<T>* robot)
    : Task<T>(3), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, robot->_nDof);
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 0.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 80.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 10.);

}

template <typename T>
CentroidLinearMomentumTask<T>::~CentroidLinearMomentumTask() {}

template <typename T>
bool CentroidLinearMomentumTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {

  Vec3<T>* pos_cmd = (Vec3<T>*)pos_des;

  // Centroidal Velocity
  Vec6<T> vG = _robot_sys->getCentroidalVelocity();

  // Position
  Vec3<T> com_pos = _robot_sys->_state.bodyPosition + _robot_sys->getComPosWorld();
  // X, Y, Z
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = (*pos_cmd)[i] - com_pos[i];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
    TK::op_cmd_[i] = _Kp[i] * TK::pos_err_[i] +
                    _Kd[i] * (TK::vel_des_[i] - vG[i+3]) + 
                    0.*TK::acc_des_[i];

  } 

  // pretty_print(TK::pos_err_, std::cout, "CLM Pos Error: ");
  // pretty_print(TK::vel_des_ - vG.tail(3), std::cout, "CLM Vel Error: ");
  // pretty_print(h, std::cout, "Centroid Momentum");
  // pretty_print(com_pos, std::cout, "CoM pos");
  // pretty_print(hdot_cmd, std::cout, "hdot cmd");
  // pretty_print(TK::op_cmd_, std::cout, "op cmd");
  // std::cout << "[CM Task] Updated Command" << std::endl;
  return true;
}

template <typename T>
bool CentroidLinearMomentumTask<T>::_UpdateTaskJacobian() {

  TK::Jt_ = ((_robot_sys->getCompositeRBI()).inverse() * _robot_sys->getCMM()).block(3,0,TK::dim_task_, _robot_sys->_nDof);

  // pretty_print(_robot_sys->getCMM(), std::cout, "Centroid Momentum Matrix");
  // pretty_print(_robot_sys->getCompositeRBI(), std::cout, "Ig");
  // pretty_print(TK::Jt_, std::cout, "Jt CM");
  // std::cout << "[CM Task] Updated Task Jacobian" << std::endl;
  return true;
}

template <typename T>
bool CentroidLinearMomentumTask<T>::_UpdateTaskJDotQdot() {

  // Mat6<T> Identity, Ig_inv, Igd;
  // Identity.setIdentity();
  // Ig_inv = (_robot_sys->getCompositeRBI()).inverse();
  // Igd = _robot_sys->getCompositeRBIdot();
  // TK::JtDotQdot_ = Ig_inv*(_robot_sys->getCmmBiasForce() - Igd*_robot_sys->getCentroidalVelocity());

  // Old
  TK::JtDotQdot_ = ((_robot_sys->getCompositeRBI()).inverse() * _robot_sys->getCmmBiasForce()).tail(3);

  // pretty_print(TK::JtDotQdot_, std::cout, "JtDotQdot");
  // pretty_print(_robot_sys->getCmmBiasForce(), std::cout, "CMM Bias Force");
  // std::cout << "[CM Task] Updated Task JDotQdot" << std::endl;
  return true;
}

template class CentroidLinearMomentumTask<double>;
template class CentroidLinearMomentumTask<float>;
