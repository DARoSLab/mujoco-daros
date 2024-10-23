#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include <FBModel/FloatingBaseModel.h>
#include <PrestoeDefinition.h>
#include <cppTypes.h>
#include <WBIC_FB/WBIC.hpp>
#include <WBIC_FB/KinWBC.hpp>

#include <lcm-cpp.hpp>
#include "wbc_test_data_lcmt.hpp"

template<typename T>
class WBC_Ctrl{
  public:
    WBC_Ctrl(const FloatingBaseModel<T> * model);
    virtual ~WBC_Ctrl();

    void run(void * input);
    void setFloatingBaseWeight(const T & weight){
      _wbic_data->_W_floating = DVec<T>::Constant(6, weight);
    }
    void setFloatingBaseWeight(const DVec<T> & weight){
      _wbic_data->_W_floating = weight;
    }
    wbc_test_data_lcmt _wbc_data_lcm;

    DVec<T> _tau_ff;
    DVec<T> _des_jpos;
    DVec<T> _des_jvel;

    std::vector<T> _Kp_joint, _Kd_joint;

  protected:
    virtual void _ContactTaskUpdate(void * input) = 0;
    virtual void _LCM_PublishData(){}
    void _UpdateParams();
    virtual void _ComputeWBC();

    KinWBC<T>* _kin_wbc;
    WBIC<T>* _wbic;
    WBIC_ExtraData<T>* _wbic_data;

    FBModelState<T>* _state;
    FloatingBaseModel<T>* _model;
    std::vector<ContactSpec<T> * > _contact_list;
    std::vector<Task<T> * > _task_list;

    DMat<T> _A;
    DMat<T> _Ainv;
    DVec<T> _grav;
    DVec<T> _coriolis;

    DVec<T> _full_config;

    unsigned long long _iter;

    lcm::LCM _wbcLCM;
};
#endif
