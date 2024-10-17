#ifndef PITCH_CONTACT_H
#define PITCH_CONTACT_H

#include <Dynamics/FloatingBaseModel.h>
#include <WBC/ContactSpec.hpp>

template <typename T>
class PitchContact : public ContactSpec<T> {
 public:
  PitchContact(const FloatingBaseModel<T>* robot, int contact_pt);
  virtual ~PitchContact();

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
