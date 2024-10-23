#ifndef MOMENTUM_TASK
#define MOMENTUM_TASK
#include <WBIC_FB/Task.hpp>

template <typename T>
class FloatingBaseModel;

template <typename T>
class MomentumTask : public Task<T> {
 public:
  MomentumTask(const FloatingBaseModel<T>*);
  virtual ~MomentumTask();
  DVec<T> _Kp_kin, _Kp, _Kd;

 protected:
  virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des);
  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate() { return true; }
  const FloatingBaseModel<T>* _robot_sys;
};
#endif