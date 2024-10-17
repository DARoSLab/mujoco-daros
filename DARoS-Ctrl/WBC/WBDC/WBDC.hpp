#ifndef WHOLE_BODY_DYNAMIC_CONTROL_H
#define WHOLE_BODY_DYNAMIC_CONTROL_H

#include <Utilities/pretty_print.h>
#include <Goldfarb_Optimizer/QuadProg++.hh>
#include <WBC/Task.hpp>
#include <WBC/WBC.hpp>
#include <WBC/ContactSpec.hpp>
#include <WBC/WBIC/WBIC.hpp>

template <typename T>
class WBDC: public WBC<T>{
  public:
    WBDC(size_t num_qdot, const std::vector<ContactSpec<T>*>* contact_list,
        const std::vector<Task<T>*>* task_list);

    virtual ~WBDC(){}
    virtual void UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
        const DVec<T>& cori, const DVec<T>& grav,
        void* extra_setting = NULL);


    virtual void MakeTorque(DVec<T>& cmd, void* extra_input = NULL);

  private:
    const std::vector<ContactSpec<T>*>* _contact_list;
    const std::vector<Task<T>*>* _task_list;

    void _SetInEqualityConstraint();
    void _ContactBuilding();

    void _GetSolution(const DVec<T> & qddot, DVec<T> & cmd);
    bool _CheckNullSpace(const DMat<T> & Npre);
    void _OptimizationPreparation();

    size_t _dim_opt;
    size_t _dim_eq_cstr; // equality constraints
    size_t _dim_ieq_cstr; // inequality constraints
    size_t dim_first_task_; // first task dimension
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

    size_t _dim_rf = 0;
    size_t _dim_Uf = 0;

    DMat<T> _Uf;
    DVec<T> _Uf_ieq_vec;

    DMat<T> _Jc;
    DVec<T> _JcDotQdot;

    DMat<T> B_;
    DVec<T> c_;
    DVec<T> task_cmd_;

    DMat<T> _eye;
    DMat<T> _eye_floating;

    void _PrintDebug(T i) {
      printf("[WBDC] %f \n", i);
    }
    size_t _dim_floating;
};

#endif
