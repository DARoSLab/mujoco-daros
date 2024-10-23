#include "BodyOriTask2.hpp"
// (Rx, Ry, Rz)

#include <Configuration.h>
#include <FBModel/FloatingBaseModel.h>
#include <pretty_print.h>


template <typename T>
BodyOriTask2<T>::BodyOriTask2(const FloatingBaseModel<T>* robot)
    : Task<T>(3), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, robot->_nDof);
  TK::Jt_.block(0, 0, 3, 3).setIdentity();
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 50.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 50.);
}

template <typename T>
BodyOriTask2<T>::~BodyOriTask2() {}

template <typename T>
bool BodyOriTask2<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {
  Quat<T>* ori_cmd = (Quat<T>*)pos_des;
  Quat<T> link_ori = (_robot_sys->_state.bodyOrientation);
  //Vec3<T> rpoyori = ori::quatToRPY(link_ori);
  Quat<T> link_ori_inv;
  link_ori_inv[0] = link_ori[0];
  link_ori_inv[1] = -link_ori[1];
  link_ori_inv[2] = -link_ori[2];
  link_ori_inv[3] = -link_ori[3];
  // link_ori_inv /= link_ori.norm();

  // Explicit because operational space is in global frame
  Quat<T> ori_err = ori::quatProduct(*ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) {
    ori_err *= (-1.);
  }
  ori_err.normalize();
  Vec3<T> ori_err_so3;
  ori::quaternionToso3(ori_err, ori_err_so3);
  SVec<T> curr_vel = _robot_sys->_state.bodyVelocity;

  // Configuration space: Local
  // Operational Space: Global
  Mat3<T> Rot = ori::quaternionToRotationMatrix(link_ori);
  Vec3<T> vel_err = Rot.transpose()*(vel_des-curr_vel.head(3));
  Mat3<T> Rotd = ori::quaternionToRotationMatrix(*ori_cmd);
  Vec3<T> omW = Rotd.transpose()*vel_des;

  // Rx, Ry, Rz
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i] * ori_err_so3[i];
    TK::vel_des_[i] = omW[i];//vel_des[i];
    TK::acc_des_[i] = acc_des[i];
    TK::op_cmd_[i] = _Kp[i] * ori_err_so3[i] + _Kd[i] * vel_err[i] + TK::acc_des_[i];
  }
   /*printf("[Body Ori Task]\n");
   pretty_print(TK::pos_err_, std::cout, "pos_err_");
   pretty_print(*ori_cmd, std::cout, "des_ori");
   pretty_print(link_ori, std::cout, "curr_ori");
   pretty_print(ori_err, std::cout, "quat_err");
   //pretty_print(Rot, std::cout, "Rot");*/
   /*Vec3<T> rpydesWBCbODYTASK=ori::quatToRPY(*ori_cmd);
   Vec3<T> rpyACTUALWBCbODYTASK=ori::quatToRPY(link_ori);
   Vec3<T> omg = curr_vel.head(3);
  std::cout<<"rpydesWBBT \t"<<rpydesWBCbODYTASK.transpose()<<"rpyrialWBBT \t"<<rpyACTUALWBCbODYTASK.transpose()<<"locome_des \t"<<vel_des.transpose()<<"ome_rial \t"<<omg.transpose()<<std::endl;
  */
  // pretty_print(vel_des, std::cout, "locome_des");
  // pretty_print(omg, std::cout, "ome_rial");
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(link_ori_inv, std::cout, "ori_inv");
  // pretty_print(ori_err, std::cout, "ori_err");
  // pretty_print(*ori_cmd, std::cout, "cmd");
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(TK::Jt_, std::cout, "Jt");

  return true;
}

template <typename T>
bool BodyOriTask2<T>::_UpdateTaskJacobian() {
  Quat<T> quat = _robot_sys->_state.bodyOrientation;
  Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
  TK::Jt_.block(0, 0, 3, 3) = Rot.transpose();
  //pretty_print(Rot, std::cout, "Rot mat");
  return true;
}

template <typename T>
bool BodyOriTask2<T>::_UpdateTaskJDotQdot() {
  return true;
}

template class BodyOriTask2<double>;
template class BodyOriTask2<float>;
