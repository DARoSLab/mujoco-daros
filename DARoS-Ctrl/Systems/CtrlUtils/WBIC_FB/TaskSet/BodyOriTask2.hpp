#ifndef BODY_ORIENTATION_TASK2
#define BODY_ORIENTATION_TASK2
#include <WBIC_FB/Task.hpp>

template <typename T>
class FloatingBaseModel;

template <typename T>
class BodyOriTask2 : public Task<T> {
 public:
  BodyOriTask2(const FloatingBaseModel<T>*);
  virtual ~BodyOriTask2();
  DVec<T> _Kp_kin, _Kp, _Kd;
 protected:
  virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des);
  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate() { return true; }
  int link_idx_;
  bool virtual_depend_;
  const FloatingBaseModel<T>* _robot_sys;
};

#endif
