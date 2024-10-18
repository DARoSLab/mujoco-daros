#include "PitchContact.hpp"
#include <Utilities/pretty_print.h>

// [ Local Ry, Fx, Fy, Fz ]
template <typename T>
PitchContact<T>::PitchContact(const FloatingBaseModel<T>* robot, int pt)
    : ContactSpec<T>(4), _max_Fz(1500.), _contact_pt(pt), _dim_U(8) {

  Contact::idx_Fz_ = 3;
  robot_sys_ = robot;
  Contact::Jc_ = DMat<T>(Contact::dim_contact_, robot->_nDof);
  Contact::JcDotQdot_ = DVec<T>::Zero(Contact::dim_contact_);
  Contact::Uf_ = DMat<T>::Zero(_dim_U, Contact::dim_contact_);
}

template <typename T>
PitchContact<T>::~PitchContact() {}

template <typename T>
bool PitchContact<T>::_UpdateJc() {
  //Contact::Jc_.template block<1, robot_sys_->_nDof>(0, 0) = 
    //(robot_sys_->_Jc_full[_contact_pt]).template block<1, robot_sys_->_nDof>(1,0);

  //Contact::Jc_.template block<3, robot_sys_->_nDof> (1, 0) = 
    //(robot_sys_->_Jc_full[_contact_pt]).template block<3, robot_sys_->_nDof>(3,0);

  //printf("contact point: %d\n", _contact_pt);

  DMat<T> JcFull = robot_sys_->_Jc_full[_contact_pt];
    size_t i = robot_sys_->_gcParent.at(_contact_pt);
  //Mat3<T> Rot = robot_sys_->_Xa[i].template block<3,3>(0,0).transpose();
  Mat3<T> Rot = robot_sys_->_Xa[i].template block<3,3>(0,0);
   //Quat<T> quat = robot_sys_->_state.bodyOrientation;
   //Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
   Mat6<T> Rot_big; Rot_big.setZero();
   Rot_big.block(0,0, 3,3) = Rot;
   Rot_big.block(3,3, 3,3) = Rot;

   JcFull = (Rot_big)*JcFull;
   //JcFull = (Rot_big.transpose())*JcFull;
  //printf("contact point: %d\n", _contact_pt);
   //pretty_print(JcFull, std::cout, "Jc full");

    
  Contact::Jc_.block(0, 0, 1, robot_sys_->_nDof) = 
    JcFull.block(1, 0, 1, robot_sys_->_nDof);

  Contact::Jc_.block(1, 0, 3, robot_sys_->_nDof) = 
    JcFull.block(3, 0, 3, robot_sys_->_nDof);


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
bool PitchContact<T>::_UpdateJcDotQdot() {
  // TEMPORARY
  Contact::JcDotQdot_.setZero();
  // pretty_print(Contact::JcDotQdot_, std::cout, "JcDotQdot");
  return true;
}

template <typename T>
bool PitchContact<T>::_UpdateUf() {
  T mu(0.4);
  T toe(0.08);
  T heel(0.05);

  Contact::Uf_ = DMat<T>::Zero(_dim_U, Contact::dim_contact_);
  // Ry(0), Fx(1), Fy(2), Fz(3)

  // Linear
  Contact::Uf_(0, 3) = 1.;  // Fz >= 0

  Contact::Uf_(1, 1) = 1.; Contact::Uf_(1, 3) = mu;
  Contact::Uf_(2, 1) = -1.; Contact::Uf_(2, 3) = mu;

  Contact::Uf_(3, 2) = 1.; Contact::Uf_(3, 3) = mu;
  Contact::Uf_(4, 2) = -1.; Contact::Uf_(4, 3) = mu;

  // Angular (Flip)
  Contact::Uf_(5, 0) = -1/toe; Contact::Uf_(5, 3) = 1;
  Contact::Uf_(6, 0) = 1/heel; Contact::Uf_(6, 3) = 1;

  // Upper bound of vertical directional reaction force
  Contact::Uf_(7, 3) = -1.;  // -Fz >= -max_Fz_
  return true;
}

template <typename T>
bool PitchContact<T>::_UpdateInequalityVector() {
  Contact::ieq_vec_ = DVec<T>::Zero(_dim_U);
  Contact::ieq_vec_[7] = -_max_Fz;
  return true;
}

template class PitchContact<double>;
template class PitchContact<float>;
