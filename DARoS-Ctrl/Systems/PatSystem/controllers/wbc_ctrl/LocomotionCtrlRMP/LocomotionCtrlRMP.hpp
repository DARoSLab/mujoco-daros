#ifdef RMPFLOW_BUILD
#ifndef PAT_RMP_LOCOMOTION_CONTROLLER
#define PAT_RMP_LOCOMOTION_CONTROLLER


#include <wbc_ctrl/WBC_Ctrl.hpp>

template<typename T>
class LocomotionCtrlRMPData{
  public:
    Vec3<T> pBody_des;
    Vec3<T> vBody_des;
    Vec3<T> aBody_des;
    Vec3<T> pBody_RPY_des;
    Vec3<T> vBody_Ori_des;

    Vec3<T> pFoot_des[pat_biped::num_legs];
    Vec3<T> vFoot_des[pat_biped::num_legs];
    Vec3<T> aFoot_des[pat_biped::num_legs];
    Vec3<T> Fr_des[pat_biped::num_legs];

    Vec2<T> contact_state;

    int lcm_publish_channel = 0;
};

template<typename T>
class LocomotionCtrlRMP: public WBC_Ctrl<T>{
  public:
    LocomotionCtrlRMP(const ControlFSMData<T> * data);
    virtual ~LocomotionCtrlRMP();

  protected:
    virtual void _ContactTaskUpdate(void * input);
    void _ParameterSetup(const PatParameters* param);
    void _CleanUp();
    virtual void _LCM_PublishData();

    LocomotionCtrlRMPData<T>* _input_data;

    Task<T>* _body_pos_task;
    Task<T>* _body_ori_task;

    Task<T>* _body_rpz_task;

    Task<T>* _foot_task[pat_biped::num_legs];
    ContactSpec<T>* _foot_contact[pat_biped::num_legs];

    Vec3<T> _Fr_result[pat_biped::num_legs];
    Quat<T> _quat_des;
    Vec3<T> _rpy_des;
    Vec3<T> _rpy_vel_des;
    Vec3<T> _rpz_des;
    Vec3<T> _rpz_vel_des;
    Vec3<T> _rpz_acc_des;
};

#endif
#endif
