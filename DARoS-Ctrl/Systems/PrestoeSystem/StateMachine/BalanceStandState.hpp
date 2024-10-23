#ifndef __BALANCED_STAND_H__
#define __BALANCED_STAND_H__

#include "State.hpp"

template<typename T> class WBC_Ctrl;
template<typename T> class PrestoeStandCtrlData;
template <typename T> class ObserverManager;
template <typename T> class Command;

template <typename T>
class BalanceStandState : public State<T> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BalanceStandState(ObserverManager<T> * obs_manager, PrestoeSystem<T> * prestoe_system);

    virtual void OnEnter();
    virtual void RunNominal();
    virtual Command<T>* GetCommand() { return _jtorque_cmd; }

  protected:
    bool _b_standing_up = true;

    void _KeepPostureStep();
    void _UpdateModel();

    WBC_Ctrl<T> * _wbc_ctrl;
    PrestoeStandCtrlData<T> * _wbc_data;

    Vec3<T> _des_com_pos; 
    Vec3<T> _ini_com_pos;
    Vec3<T> _ini_body_ori_rpy;
    Vec3<T> _ini_body_pos;
    DVec<T> _ini_jpos;
    Vec3<T> _mid_pos_cps;
    T _body_weight;
    
    Command<T>* _jtorque_cmd;

    FBModelState<T> _fb_state;
    FloatingBaseModel<T>* _fb_model;
};



#endif // __BALANCED_STAND_H__