#include "LineContactWrenchCone.hpp"
#include <pretty_print.h>

template <typename T>
LineContactWrenchCone<T>::LineContactWrenchCone(const FloatingBaseModel<T>* robot, int pt1, int pt2) : ContactSpec<T>(5), _max_Fz(3000.), _contact_pt1(pt1), _contact_pt2(pt2), _dim_U(7) {
  Contact::idx_Fz_ = 4;
  robot_sys_ = robot;
  Contact::Jc_ = DMat<T>(Contact::dim_contact_, robot->_nDof);
  Contact::JcDotQdot_ = DVec<T>::Zero(Contact::dim_contact_);
  Contact::Uf_ = DMat<T>::Zero(_dim_U, Contact::dim_contact_);
  if(robot_sys_->_gcParent.at(_contact_pt1)!=robot_sys_->_gcParent.at(_contact_pt2)){
    throw std::runtime_error("Contact Points inconsistent!");
  }
}

template <typename T>
LineContactWrenchCone<T>::~LineContactWrenchCone() {}

template <typename T>
bool LineContactWrenchCone<T>::_UpdateJc() {

  DMat<T> JcFull1 = robot_sys_->_Jc_full[_contact_pt1];
  DMat<T> JcFull2 = robot_sys_->_Jc_full[_contact_pt2];
  DMat<T> JcFull = (JcFull1+JcFull2)*(T)0.5;
  size_t i = robot_sys_->_gcParent.at(_contact_pt1);
  Mat3<T> Rot = robot_sys_->_Xa[i].template block<3,3>(0,0);
  /*std::cout<<"omcrc"<<-(Rot-Rotprev)/0.004*((Rot+Rotprev).transpose())<<std::endl;
  std::cout<<"omcr2"<<robot_sys_->_v[i].transpose()<<std::endl;
  Rotprev=Rot;*/
  DVec<T> qdotn(robot_sys_->_nDof);
  for (size_t ig(0); ig < 6; ++ig) qdotn[ig] = robot_sys_->_state.bodyVelocity[ig];
  for (size_t ig(0); ig < robot_sys_->_nDof - 6; ++ig) qdotn[ig + 6] = robot_sys_->_state.qd[ig];
  //std::cout<<"ggwp: "<<-robot_sys_->_v[i].template head<3>().cross(Rot*JcFull.block(0,0,3,robot_sys_->_nDof)*qdotn)<<std::endl;
  /*Mat6<T> Rot_big; Rot_big.setZero();
  Rot_big.block(0,0,3,3) = Rot;
  Rot_big.block(3,3,3,3) = Rot;
  JcFull = (Rot_big)*JcFull;*/

  //std::cout<<"R1"<<(Rot*(robot_sys_->_pGC[_contact_pt1]-robot_sys_->_pGC[_contact_pt2])).transpose();

  
  /*std::cout<<"JcFull2: "<< JcFull2<<std::endl;
  std::cout<<"JcFull1: "<< JcFull1<<std::endl;*/
  Contact::Jc_.block(0,0,2,robot_sys_->_nDof) = Rot.block(1,0,2,3)*JcFull.block(0,0,3,robot_sys_->_nDof);
  Contact::Jc_.block(2,0,3,robot_sys_->_nDof) = JcFull.block(3,0,3,robot_sys_->_nDof);
  

  /*Quat<T> quat = robot_sys_->_state.bodyOrientation;
  Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
  size_t i = robot_sys_->_gcParent.at(_contact_pt1);
  //Mat3<T> Rot = robot_sys_->_Xa[i].template block<3,3>(0,0);
  Mat6<T> Rot_big; Rot_big.setZero();
  Rot_big.block(0,0, 3,3) = Rot;
  Rot_big.block(3,3, 3,3) = Rot;
  JcFull1 = (Rot_big)*JcFull1;
  JcFull2 = (Rot_big)*JcFull2;
  std::cout<<"JcFull2: "<< JcFull2<<std::endl;
  std::cout<<"JcFull1: "<< JcFull1<<std::endl;*/

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
bool LineContactWrenchCone<T>::_UpdateJcDotQdot() {
  size_t i = robot_sys_->_gcParent.at(_contact_pt1);
  Mat3<T> Rot = robot_sys_->_Xa[i].template block<3,3>(0,0);
  DMat<T> Jcdqd_f1 = robot_sys_->_Jcdqd_full[_contact_pt1];
  DMat<T> Jcdqd_f2 = robot_sys_->_Jcdqd_full[_contact_pt2];
  DMat<T> JcdqdFull = (Jcdqd_f1+Jcdqd_f2)*(T)0.5;
  Contact::JcDotQdot_.setZero();
  Contact::JcDotQdot_.block(0,0,2,1) = Rot.block(1,0,2,3)*JcdqdFull.block(0,0,3,1);
  Contact::JcDotQdot_.block(2,0,3,1) = JcdqdFull.block(3,0,3,1);
  //pretty_print(Contact::JcDotQdot_, std::cout, "JcDotQdot");
  return true;
}

template <typename T>
bool LineContactWrenchCone<T>::_UpdateUf() {
  T mu(0.4);
  T toe(0.08);
  T heel(0.05);
  T Xdis=(toe+heel)/2;

  Contact::Uf_ = DMat<T>::Zero(_dim_U, Contact::dim_contact_);
  this->Uf_(0, 2) = 1.; this->Uf_(0, 4) = mu;
  this->Uf_(1, 2) = -1.; this->Uf_(1, 4) = mu;
  this->Uf_(2, 0) = -mu; this->Uf_(2, 1) = -1; this->Uf_(2, 3) = -Xdis; this->Uf_(2, 4) = mu*Xdis;
  this->Uf_(3, 0) = -mu; this->Uf_(3, 1) = 1; this->Uf_(3, 3) = Xdis; this->Uf_(3, 4) = mu*Xdis;
  this->Uf_(4, 0) = mu; this->Uf_(4, 1) = 1; this->Uf_(4, 3) = -Xdis; this->Uf_(4, 4) = mu*Xdis;
  this->Uf_(5, 0) = mu; this->Uf_(5, 1) = -1; this->Uf_(5, 3) = Xdis; this->Uf_(5, 4) = mu*Xdis;
  this->Uf_(6, 4) = -1.;  // -Fz >= -max_Fz_
  size_t i = robot_sys_->_gcParent.at(_contact_pt1);
  Mat3<T> Rot = robot_sys_->_Xa[i].template block<3,3>(0,0);
  Mat5<T> Rot_big; Rot_big.setIdentity();
  Rot_big.block(2,2,3,3) = Rot;
  Contact::Uf_=Contact::Uf_*Rot_big;
  //this->Uf_(8, 0) = -1; this->Uf_(3, 4) = Xdis;
  //this->Uf_(9, 0) = 1; this->Uf_(4, 4) = Xdis;
  return true;
}

template <typename T>
bool LineContactWrenchCone<T>::_UpdateInequalityVector() {
  Contact::ieq_vec_ = DVec<T>::Zero(_dim_U);
  Contact::ieq_vec_[6] = -_max_Fz;
  return true;
}

template class LineContactWrenchCone<double>;
template class LineContactWrenchCone<float>;
