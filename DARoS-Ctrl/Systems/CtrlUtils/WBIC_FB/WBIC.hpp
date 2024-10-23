#ifndef WHOLE_BODY_IMPULSE_CONTROL_H
#define WHOLE_BODY_IMPULSE_CONTROL_H

#include <pretty_print.h>
#include <WBIC_FB/Goldfarb_Optimizer/QuadProg++.hpp>
#include <WBIC_FB/ContactSpec.hpp>
#include <WBIC_FB/Task.hpp>
#include <pseudoInverse.h>

template <typename T>
class WBIC_ExtraData {
 public:
  // Output
  DVec<T> _opt_result;
  DVec<T> _qddot;
  DVec<T> _Fr;

  // Input
  DVec<T> _W_floating;
  DVec<T> _W_rf;

  WBIC_ExtraData() {}
  ~WBIC_ExtraData() {}
};

template <typename T>
class WBIC {
  public:
    WBIC(size_t num_qdot, const std::vector<ContactSpec<T>*>* contact_list,
        const std::vector<Task<T>*>* task_list);
    ~WBIC() {}

    void UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
                      const DVec<T>& cori, const DVec<T>& grav,
                      void* extra_setting = NULL);

    void MakeTorque(DVec<T>& cmd, void* extra_input = NULL);
    void FindConfigurationAndTorque(const DVec<T> & curr_config, 
                                    DVec<T>& jpos_cmd, DVec<T> & jvel_cmd, DVec<T> & cmd, 
                                    void* extra_input);


  private:
    void _WeightedInverse(const DMat<T>& J, const DMat<T>& Winv, DMat<T>& Jinv,
        double threshold = 0.0001) {
      DMat<T> lambda(J * Winv * J.transpose());
      DMat<T> lambda_inv;
      pseudoInverse(lambda, threshold, lambda_inv);
      Jinv = Winv * J.transpose() * lambda_inv;
    }

    size_t num_act_joint_;
    size_t num_qdot_;

    DMat<T> Sa_;  // Actuated joint
    DMat<T> Sv_;  // Virtual joint

    DMat<T> A_;
    DMat<T> Ainv_;
    DVec<T> cori_;
    DVec<T> grav_;

    bool b_updatesetting_;
    bool b_internal_constraint_;

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
