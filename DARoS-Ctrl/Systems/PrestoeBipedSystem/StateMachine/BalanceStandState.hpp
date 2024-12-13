#ifndef __BALANCED_STAND_H__
#define __BALANCED_STAND_H__

#include "State.hpp"
#include <FBModel/FloatingBaseModel.h>

template<typename T> class WBC_Ctrl;

template<typename T> class PrestoeStandCtrlData;
template <typename T> class ObserverManager;
template <typename T> class Command;

template <typename T>
class BalanceStandState : public State<T> {
  public:
    BalanceStandState(ObserverManager<T> * obs_manager, PrestoeBipedSystem<T> * prestoe_system);

    virtual void OnEnter();
    virtual void RunNominal();
    virtual Command<T>* GetCommand() { return _jtorque_pos_cmd; }

  protected:
    ObserverManager<T>* _obs_manager;

    void _ReadConfig(const std::string & file_name);
    void _KeepPostureStep();
    void _UpdateModel();
    void _UpdateCommand();

    WBC_Ctrl<T> * _wbc_ctrl;
    PrestoeStandCtrlData<T> * _wbc_data;

    T _targetHeight;
    T _x_pos_offset;
    T _z_swing_amp;
    T _z_swing_freq;

    Vec3<T> _ini_body_ori_rpy;
    Vec3<T> _ini_body_pos;
    DVec<T> _ini_jpos;
    Vec3<T> _mid_pos_cps;
    T _body_weight;
    
    Command<T>* _jtorque_pos_cmd;
    DVec<T> _Kp, _Kd;

    FBModelState<T> _fb_state;
    FloatingBaseModel<T> _fb_model;
};



#endif // __BALANCED_STAND_H__