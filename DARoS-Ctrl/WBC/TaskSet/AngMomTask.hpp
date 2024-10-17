#ifndef ANGMOM_TASK
#define ANGMOM_TASK
#include <WBC/Task.hpp>

template <typename T>
class FloatingBaseModel;

template <typename T>
class AngMomTask : public Task<T> {
 public:
  AngMomTask(const FloatingBaseModel<T>*);
  virtual ~AngMomTask();
  DVec<T> _Kp;
 protected:
  virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& L_des, const DVec<T>& Ld_des);
  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate() { return true; }
  int link_idx_;
  bool virtual_depend_;
  const FloatingBaseModel<T>* _robot_sys;
};
#endif