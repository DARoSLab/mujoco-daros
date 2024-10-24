#ifndef __BOX_PICKUP_STATE_H__
#define __BOX_PICKUP_STATE_H__

#include "State.hpp"
#include <FBModel/FloatingBaseModel.h>

template<typename T> class WBC_Ctrl;
template<typename T> class PrestoeBoxPickupCtrlData;
template <typename T> class ObserverManager;
template <typename T> class Command;

template <typename T>
class BoxPickupState : public State<T> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BoxPickupState(ObserverManager<T> * obs_manager, PrestoeSystem<T> * prestoe_system);

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
    PrestoeBoxPickupCtrlData<T> * _wbc_data;

    Vec3<T> _des_com_pos; 
    Vec3<T> _ini_body_ori_rpy;
    Vec3<T> _ini_com;
    Vec3<T> _ini_body_pos;
    DVec<T> _ini_jpos;
    Vec3<T> _mid_pos_cps;


    Command<T>* _jtorque_pos_cmd;
    DVec<T> _Kp, _Kd;

    FBModelState<T> _fb_state;
    FloatingBaseModel<T> _fb_model;
};



#endif // __BALANCED_STAND_H__