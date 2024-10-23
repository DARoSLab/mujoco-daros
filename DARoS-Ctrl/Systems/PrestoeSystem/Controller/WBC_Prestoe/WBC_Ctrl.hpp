#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include <FBModel/FloatingBaseModel.h>
#include <Robots/PrestoeDefinition.h>
#include <cppTypes.h>
#include <WBIC_FB/WBIC.hpp>

#include <lcm-cpp.hpp>
#include "wbc_test_data_lcmt.hpp"

template<typename T>
class WBC_Ctrl{
  public:
    WBC_Ctrl(const FloatingBaseModel<T> * model);
    virtual ~WBC_Ctrl();

    void run(void * input, ControlFSMData_Staccatoe<T> & data);
    void setFloatingBaseWeight(const T & weight){
      _wbic_data->_W_floating = DVec<T>::Constant(6, weight);
    }
    void setFloatingBaseWeight(const DVec<T> & weight){
      _wbic_data->_W_floating = weight;
    }
    bool trigland = false; // to export to cMPC if needed (e.g. landing)
    wbc_test_data_lcmt _wbc_data_lcm;

  protected:
    virtual void _ContactTaskUpdate(void * input, ControlFSMData_Staccatoe<T> & data) = 0;
    virtual void _LCM_PublishData(){}
    void _UpdateModel(const StateEstimate<T> & state_est, ControlFSMData_Staccatoe<T> & fsm_data);
    void _UpdateJointCMD(ControlFSMData_Staccatoe<T> & data);
    virtual void _ComputeWBC();

    KinWBC<T>* _kin_wbc;
    WBC<T>* _wbic;
    WBIC_ExtraData<T>* _wbic_data;

    FBModelState<T> _state;
    FloatingBaseModel<T>* _model;
    std::vector<ContactSpec<T> * > _contact_list;
    std::vector<Task<T> * > _task_list;

    DMat<T> _A;
    DMat<T> _Ainv;
    DVec<T> _grav;
    DVec<T> _coriolis;

    DVec<T> _full_config;
    DVec<T> _tau_ff;
    DVec<T> _des_jpos;
    DVec<T> _des_jvel;

    std::vector<T> _Kp_joint, _Kd_joint;
    //std::vector<T> _Kp_joint_swing, _Kd_joint_swing;

    unsigned long long _iter;

    lcm::LCM _wbcLCM;

    DMat<T> _CorMat;
    DVec<T> _pold;
    DVec<T> _taudistold;
    Vec10<T> forsol; //only for lcm, change
};
#endif
