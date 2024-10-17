#ifndef WHOLE_BODY_IMPULSE_CONTROL_TEST_H
#define WHOLE_BODY_IMPULSE_CONTROL_TEST_H

#include <Utilities/pretty_print.h>
#include <Goldfarb_Optimizer/QuadProg++.hh>
#include <WBC/ContactSpec.hpp>
#include <WBC/Task.hpp>
#include <WBC/WBC.hpp>
#include "WBIC.hpp"

template <typename T>
class WBIC_test : public WBC<T> {
 public:
  WBIC_test(size_t num_qdot, const std::vector<ContactSpec<T>*>* contact_list,
       const std::vector<Task<T>*>* task_list);
  virtual ~WBIC_test() {}

  virtual void UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
                             const DVec<T>& cori, const DVec<T>& grav,
                             void* extra_setting = NULL);

  virtual void MakeTorque(DVec<T>& cmd, void* extra_input = NULL);

  size_t _rf_computation_idx = 100;
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

  WBIC_ExtraData<T>* _data;

  GolDIdnani::GVect<double> z;
  // Cost
  GolDIdnani::GMatr<double> G;
  GolDIdnani::GVect<double> g0;

  // Equality
  GolDIdnani::GMatr<double> CE;
  GolDIdnani::GVect<double> ce0;

  // Inequality
  GolDIdnani::GMatr<double> CI;
  GolDIdnani::GVect<double> ci0;

  DMat<T> _dyn_CE;
  DVec<T> _dyn_ce0;
  DMat<T> _dyn_CI;
  DVec<T> _dyn_ci0;

  DMat<T> _eye;
  DMat<T> _eye_floating;

  DMat<T> _S_delta;
  DMat<T> _Uf;
  DVec<T> _Uf_ieq_vec;

  DMat<T> _Jc;
  DVec<T> _JcDotQdot;
  DVec<T> _Fr_des;

  DMat<T> _B;
  DVec<T> _c;
  DVec<T> task_cmd_;
};

#endif
