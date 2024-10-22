#ifndef GUIDE_DOG_LOCOMOTION_CONTROLLER
#define GUIDE_DOG_LOCOMOTION_CONTROLLER


#include <wbc_ctrl/WBC_Ctrl.hpp>

template<typename T>
class LocomotionCtrlData{
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
class LocomotionCtrl: public WBC_Ctrl<T>{
  public:
    LocomotionCtrl(const ControlFSMData<T> * data);
    virtual ~LocomotionCtrl();

  protected:
    virtual void _ContactTaskUpdate(void * input);
    void _ParameterSetup(const PatParameters* param);
    void _ParameterSetup(const std::string & file);
    void _CleanUp();
    virtual void _LCM_PublishData();

    LocomotionCtrlData<T>* _input_data;

    Task<T>* _body_pos_task;
    Task<T>* _body_ori_task;
    Task<T>* _body_z_task;

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

    std::vector<double> _Kp_joint, _Kd_joint, _Kp_body, _Kd_body, _Kp_foot, _Kd_foot;
    std::vector<double> _Kp_ori, _Kd_ori;


};

#endif
