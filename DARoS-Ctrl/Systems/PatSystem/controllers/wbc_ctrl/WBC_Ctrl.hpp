#ifndef PAT_WBC_CONTROLLER_H
#define PAT_WBC_CONTROLLER_H

#include <state_machine/ControlFSMData.h>
#include <dynamics/FloatingBaseModel.h>
#include <robots/PatBiped.h>
#include <cppTypes.h>
#include <WBC/WBIC/WBIC.hpp>
#include <WBC/WBDC/WBDC.hpp>
#include <WBC/WBIC/KinWBC.hpp>

#include <lcm-cpp.hpp>
#include "wbc_test_data_lcmt.hpp"

template<typename T>
class WBC_Ctrl{
  public:
    WBC_Ctrl(const ControlFSMData<T> * data);
    virtual ~WBC_Ctrl();

    void run(void * input, ControlFSMData<T> & data);
    void setFloatingBaseWeight(const T & weight){
      _wbic_data->_W_floating = DVec<T>::Constant(6, weight);
    }
    void setReactionForceWeight(const T & weight){
      _wbic_data->_W_rf = DVec<T>::Constant(12, weight);
    }
    void setFloatingBaseWeight(const DVec<T> & weight){
      _wbic_data->_W_floating = weight;
    }
    bool trigland = false; // to export to cMPC if needed (e.g. landing)
    const FloatingBaseModel<T> & getModel(){ return _model; }
    const FloatingBaseModel<T> * getModelPtr(){ return &_model; }
    void updateRMP(DMat<T> M, DVec<T> f){
      _wbc->setMRMP(M);
      _wbc->setFRMP(f);
    }
  protected:
    void _EstimateExternalForce();

    virtual void _ContactTaskUpdate(void * input) = 0;
    virtual void _LCM_PublishData(){}
    void _UpdateModel();
    void _UpdateModel_local();
    void _UpdateLegCMD();
    void _ComputeWBC();

    KinWBC<T>* _kin_wbc;
    WBIC<T>* _wbc;
    WBIC_ExtraData<T>* _wbic_data;

    FloatingBaseModel<T> _model;
    const ControlFSMData<T> * _data;
    std::vector<ContactSpec<T> * > _contact_list;
    std::vector<Task<T> * > _task_list;
    bool ESTOP = false;
    DMat<T> _A;
    DMat<T> _Ainv;
    DVec<T> _grav;
    DVec<T> _coriolis;

    DMat<T> _Jc_full;
    DVec<T> _tau_dist;
    DVec<T> _tau_dist_old;
    DVec<T> _p_old;

    FBModelState<T> _state;

    DVec<T> _full_config;
    DVec<T> _tau_ff;
    DVec<T> _des_jpos;
    DVec<T> _des_jvel;


    Vec3<T> _F_ext_est;
    Vec3<T> _F_ext_est_DOB;

    Vec3<T> rpy_ini;
    Vec4<T> _ori_ini_inv;
    bool first_visit = true;

    std::vector<T> _Kp_joint, _Kd_joint;
    //std::vector<T> _Kp_joint_swing, _Kd_joint_swing;

    unsigned long long _iter;

    lcm::LCM _wbcLCM;
    wbc_test_data_lcmt _wbc_data_lcm;

    void _EstimateContactForces();
    void _LoadJointLimitParams(const std::string& file);

    std::vector<double> _jpos_min, _jpos_max;

    void _UpdateStateEst(ControlFSMData<T> & data){       // update stateEst values from WBC if needed
      data._stateEstimator->setContactForces(forsol);     // pass in estimated contact forces to state estimation
    }

    DVec<T> _tauest;//12
    Vec12<T> forsol;
    DMat<T> _CorMat;
};
#endif
