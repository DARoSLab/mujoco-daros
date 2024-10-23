#ifndef LEFT_KNEE_ANKLE_JPOS_TASK_Cheetah
#define LEFT_KNEE_ANKLE_JPOS_TASK_Cheetah

#include <WBIC_FB/Task.hpp>

template <typename T>
class FloatingBaseModel;

template <typename T>
class LeftKneeAnkleJPosTask : public Task<T> {
 public:
  LeftKneeAnkleJPosTask(const FloatingBaseModel<T>*);
  virtual ~LeftKneeAnkleJPosTask();

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
  const FloatingBaseModel<T>* robot_sys_;
  const size_t Nj = 2; // nb joints for this jpos task.

};

#endif
