#ifndef LINECWC_H
#define LINECWC_H

#include <Dynamics/FloatingBaseModel.h>
#include <WBC/ContactSpec.hpp>

template <typename T>
class LineContactWrenchCone : public ContactSpec<T> {
 public:
  LineContactWrenchCone(const FloatingBaseModel<T>* robot, int contact_pt1, int contact_pt2);
  virtual ~LineContactWrenchCone();
  void setMaxFz(T max_fz) { _max_Fz = max_fz; }
  void setRFDesired(const DVec<T>& Fr_des) {
    size_t i = robot_sys_->_gcParent.at(_contact_pt1);
    Mat3<T> Rot = robot_sys_->_Xa[i].template block<3,3>(0,0);
    Contact::Fr_des_.resize(5,1);
    Contact::Fr_des_.block(0,0,2,1)=Rot.block(1,0,2,3)*Fr_des.block(0,0,3,1);
    Contact::Fr_des_.block(2,0,3,1)=Fr_des.block(3,0,3,1);
    //std::cout<<"Contact::Wr_des_"<<Contact::Fr_des_.transpose()<<std::endl;
    }

 protected:
  T _max_Fz;
  int _contact_pt1;
  int _contact_pt2;
  int _dim_U;
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();
  const FloatingBaseModel<T>* robot_sys_;
};

#endif
