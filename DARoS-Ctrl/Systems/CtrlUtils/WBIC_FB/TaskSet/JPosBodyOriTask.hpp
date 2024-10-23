#ifndef JPOS_BODY_ORIENTATION_TASK
#define JPOS_BODY_ORIENTATION_TASK
// (Rx, Ry, Rz, jpos)
#include <WBIC_FB/Task.hpp>

template <typename T>
class FloatingBaseModel;

template <typename T>
class JPosBodyOriTask : public Task<T> {
 public:
  JPosBodyOriTask(const FloatingBaseModel<T>*);
  virtual ~JPosBodyOriTask();

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

  int link_idx_;
  bool virtual_depend_;
  const FloatingBaseModel<T>* _robot_sys;
};

#endif
