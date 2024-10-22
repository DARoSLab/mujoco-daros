#include "PitchYawContact.hpp"
#include <Utilities/pretty_print.h>

// [ Local Ry, Local Rz, Fx, Fy, Fz ]
template <typename T>
PitchYawContact<T>::PitchYawContact(const FloatingBaseModel<T>* robot, int pt)
    : ContactSpec<T>(5), _max_Fz(1500.), _contact_pt(pt), _dim_U(12) {

  Contact::idx_Fz_ = 4;
  robot_sys_ = robot;
  Contact::Jc_ = DMat<T>(Contact::dim_contact_, robot->_nDof);
  Contact::JcDotQdot_ = DVec<T>::Zero(Contact::dim_contact_);
  Contact::Uf_ = DMat<T>::Zero(_dim_U, Contact::dim_contact_);
}

template <typename T>
PitchYawContact<T>::~PitchYawContact() {}

template <typename T>
bool PitchYawContact<T>::_UpdateJc() {

  DMat<T> JcFull = robot_sys_->_Jc_full[_contact_pt];
  size_t i = robot_sys_->_gcParent.at(_contact_pt);

  Mat3<T> Rot = robot_sys_->_Xa[i].template block<3,3>(0,0);
  Mat6<T> Rot_big; Rot_big.setZero();
  Rot_big.block(0,0, 3,3) = Rot;
  Rot_big.block(3,3, 3,3) = Rot;

  JcFull = (Rot_big)*JcFull;
    
  Contact::Jc_.block(0, 0, 2, robot_sys_->_nDof) = JcFull.block(1, 0, 2, robot_sys_->_nDof);

  Contact::Jc_.block(2, 0, 3, robot_sys_->_nDof) = JcFull.block(3, 0, 3, robot_sys_->_nDof);

  // Quat<T> quat = robot_sys_->_state.bodyOrientation;
  // Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
  // Contact::Jc_.block(0,3, 3,3) = Rot*Contact::Jc_.block(0,3,3,3);

  // Contact::Jc_.block(0,0, 3,3) = Rot.transpose()*Contact::Jc_.block(0,0,3,3);
  //pretty_print(Rot, std::cout, "body ori");
  //pretty_print(Contact::Jc_, std::cout, "Jc");
  //pretty_print(robot_sys_->_Jc_full[_contact_pt], std::cout, "Jc full");
  return true;
}

template <typename T>
bool PitchYawContact<T>::_UpdateJcDotQdot() {
  // TEMPORARY
  Contact::JcDotQdot_.setZero();
  // pretty_print(Contact::JcDotQdot_, std::cout, "JcDotQdot");
  return true;
}

template <typename T>
bool PitchYawContact<T>::_UpdateUf() {
  T mu(0.4);
  T toe(0.08);
  T heel(0.05);

  Contact::Uf_ = DMat<T>::Zero(_dim_U, Contact::dim_contact_);
  // Ry, Rz, Fx, Fy, Fz

  // Linear
  this->Uf_(0, 4) = 1.;

  this->Uf_(1, 2) = 1.; this->Uf_(1, 4) = mu;
  this->Uf_(2, 2) = -1.; this->Uf_(2, 4) = mu;

  this->Uf_(3, 3) = 1.; this->Uf_(3, 4) = mu;
  this->Uf_(4, 3) = -1.; this->Uf_(4, 4) = mu;

  // Angular (Flip)
  this->Uf_(5, 0) = -1/toe; this->Uf_(5, 4) = 1;
  this->Uf_(6, 0) = 1/heel; this->Uf_(6, 4) = 1;

  // Yaw
  this->Uf_(7, 0) = -mu/toe; this->Uf_(7, 1) = -1/toe; this->Uf_(7, 3) = -1; this->Uf_(7, 4) = mu;
  this->Uf_(8, 0) = -mu/toe; this->Uf_(8, 1) = 1/toe; this->Uf_(8, 3) = 1; this->Uf_(8, 4) = mu;

  this->Uf_(9, 0) = mu/heel; this->Uf_(9, 1) = 1/heel; this->Uf_(9, 3) = -1; this->Uf_(9, 4) = mu;
  this->Uf_(10, 0) = mu/heel; this->Uf_(10, 1) = -1/heel; this->Uf_(10, 3) = 1; this->Uf_(10, 4) = mu;
  
  // Upper bound of vertical directional reaction force
  this->Uf_(11, 3) = -1.;  // -Fz >= -max_Fz_
  return true;
}

template <typename T>
bool PitchYawContact<T>::_UpdateInequalityVector() {
  Contact::ieq_vec_ = DVec<T>::Zero(_dim_U);
  Contact::ieq_vec_[11] = -_max_Fz;
  return true;
}

template class PitchYawContact<double>;
template class PitchYawContact<float>;
