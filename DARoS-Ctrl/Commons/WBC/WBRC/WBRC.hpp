#ifndef WHOLE_BODY_RELAXED_CONTROL_H
#define WHOLE_BODY_RELAXED_CONTROL_H

#include <Utilities/pretty_print.h>
#include <Goldfarb_Optimizer/QuadProg++.hh>
#include <WBC/ContactSpec.hpp>
#include <WBC/WBC.hpp>
#include <WBC/WBIC/WBIC.hpp>


template <typename T>
class WBRC : public WBC<T> {
 public:
  WBRC(size_t num_qdot, const std::vector<ContactSpec<T>*>* contact_list, const std::vector<Task<T>*>* task_list);
  virtual ~WBRC() {}

  virtual void UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv, const DVec<T>& cori, const DVec<T>& grav, void* extra_setting = NULL);
  virtual void MakeTorque(DVec<T>& cmd, void* extra_input = NULL) {};
  void MakeTorqueWBRC(DVec<T>& cmd, void* extra_input = NULL);
  bool _DimRF(){return _dim_rf>0;};
  bool isFBcon = false;

 private:
  const std::vector<ContactSpec<T>*>* _contact_list;
  const std::vector<Task<T>*>* _task_list;

  void _SetEqualityConstraint(const DVec<T>& qddot);
  void _SetInEqualityConstraint();
  void _ContactBuilding();

  void _GetSolution(const DVec<T>& qddot, DVec<T>& cmd);
  void _SetCost();
  void _SetOptimizationSize();

  size_t _dim_opt;      // Contact pt delta, First task delta, reaction force
  size_t _dim_eq_cstr;  // equality constraints
  size_t _dim_rf;  // inequality constraints
  size_t _dim_Uf;
  size_t _dim_floating;

  WBIC_ExtraData<T>* wbc_data;

  GolDIdnani::GVect<double> z;
  GolDIdnani::GMatr<double> G;
  GolDIdnani::GVect<double> g0;

  GolDIdnani::GMatr<double> CE;
  GolDIdnani::GVect<double> ce0;

  GolDIdnani::GMatr<double> CI;
  GolDIdnani::GVect<double> ci0;

  DMat<T> _dyn_CE;
  DVec<T> _dyn_ce0;
  DMat<T> _dyn_CI;
  DVec<T> _dyn_ci0;

  DMat<T> _eye;

  DMat<T> _Uf;
  DVec<T> _Uf_ieq_vec;

  DMat<T> _Jc;
  //DMat<T> _JcFBD;
  DVec<T> _JcDotQdot;
  DVec<T> _Fr_des;
};

#endif
