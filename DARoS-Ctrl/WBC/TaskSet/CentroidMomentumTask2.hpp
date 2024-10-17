#ifndef CENTROID_MOMENTUM_TASK2
#define CENTROID_MOMENTUM_TASK2
#include <WBC/Task.hpp>

template <typename T>
class FloatingBaseModel;

template <typename T>
class CentroidMomentumTask2 : public Task<T> {
 public:
  CentroidMomentumTask2(const FloatingBaseModel<T>*);
  virtual ~CentroidMomentumTask2();

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
