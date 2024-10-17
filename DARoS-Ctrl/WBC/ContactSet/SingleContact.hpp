#ifndef SINGLE_CONTACT_H
#define SINGLE_CONTACT_H

#include <Dynamics/FloatingBaseModel.h>
#include <WBC/ContactSpec.hpp>

template <typename T>
class SingleContact : public ContactSpec<T> {
 public:
  SingleContact(const FloatingBaseModel<T>* robot, int contact_pt);
  virtual ~SingleContact();

  void setMaxFz(T max_fz) { _max_Fz = max_fz; }
  void setMinFz(T min_fz) { _min_Fz = min_fz; }

 protected:
  T _max_Fz, _min_Fz;
  int _contact_pt;
  int _dim_U;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const FloatingBaseModel<T>* robot_sys_;
};

#endif
