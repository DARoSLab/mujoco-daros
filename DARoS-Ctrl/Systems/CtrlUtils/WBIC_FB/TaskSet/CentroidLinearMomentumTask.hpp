#ifndef CENTROID_LINEAR_MOMENTUM_TASK
#define CENTROID_LINEAR_MOMENTUM_TASK
#include <WBIC_FB/Task.hpp>

template <typename T>
class FloatingBaseModel;

template <typename T>
class CentroidLinearMomentumTask : public Task<T> {
 public:
  CentroidLinearMomentumTask(const FloatingBaseModel<T>*);
  virtual ~CentroidLinearMomentumTask();

  DVec<T> _Kp_kin;
  DVec<T> _Kp, _Kd;

 protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                              const DVec<T>& acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate() { return true; }

  const FloatingBaseModel<T>* _robot_sys;
};

#endif
