#ifndef PITCH_YAW_CONTACT_H
#define PITCH_YAW_CONTACT_H

#include <FBModel/FloatingBaseModel.h>
#include <WBIC_FB/ContactSpec.hpp>

template <typename T>
class PitchYawContact : public ContactSpec<T> {
 public:
  PitchYawContact(const FloatingBaseModel<T>* robot, int contact_pt);
  virtual ~PitchYawContact();

  void setMaxFz(T max_fz) { _max_Fz = max_fz; }

 protected:
  T _max_Fz;
  int _contact_pt;
  int _dim_U;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const FloatingBaseModel<T>* robot_sys_;
};

#endif
