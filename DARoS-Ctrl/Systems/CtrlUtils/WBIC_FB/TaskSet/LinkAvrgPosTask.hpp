#ifndef LINK_AVRG_POS_TASK
#define LINK_AVRG_POS_TASK
#include <WBC/Task.hpp>

template <typename T>
class FloatingBaseModel;

template <typename T>
class LinkAvrgPosTask : public Task<T> {
 public:
  LinkAvrgPosTask(const FloatingBaseModel<T>*, int link_idx1, int link_idx2, 
  bool virtual_depend = true);
  virtual ~LinkAvrgPosTask();
  DVec<T> _Kp, _Kd, _Kp_kin;
 protected:
  virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des);
  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate() { return true; }
  const FloatingBaseModel<T>* robot_sys_;
  int link_idx1_, link_idx2_;
  bool virtual_depend_;
};

#endif
