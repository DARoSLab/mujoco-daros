#ifndef LOCALBODY_ORIENTATION_TASK
#define LOCALBODY_ORIENTATION_TASK
#include <WBC/Task.hpp>

template <typename T>
class FloatingBaseModel;

template <typename T>
class LocalBOriTask : public Task<T> {
 public:
  LocalBOriTask(const FloatingBaseModel<T>*);
  virtual ~LocalBOriTask();
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
