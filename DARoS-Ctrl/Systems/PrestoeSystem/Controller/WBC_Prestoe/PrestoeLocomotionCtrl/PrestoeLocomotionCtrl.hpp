#ifndef Staccatoe_Locomotion_CONTROLLER
#define Staccatoe_Locomotion_CONTROLLER

#include <wbc_ctrl/WBC_Ctrl.hpp>

template<typename T>
class StaccatoeLocomotionCtrlData{
  public:
    Vec3<T> pBody_des;
    Vec3<T> vBody_des;
    Vec3<T> aBody_des;
    Vec3<T> pBody_RPY_des;
    Vec3<T> vBody_Ori_des;

    Vec3<T> pFoot_des;
    Vec3<T> vFoot_des;
    Vec3<T> aFoot_des;
 
    Vec3<T> Fr_des[staccatoe_contact::num_foot_contact];
    Vec5<T> contact_state;
    DVec<T> jpos_des = DVec<T>::Zero(staccatoe::num_act_joint);
};

template<typename T>
class StaccatoeLocomotionCtrl: public WBC_Ctrl<T>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StaccatoeLocomotionCtrl(const FloatingBaseModel<T> * model);
    virtual ~StaccatoeLocomotionCtrl();

  protected:
    virtual void _ContactTaskUpdate(void * input, ControlFSMData_Staccatoe<T> & data);
    void _ParameterSetup(const StaccatoeParameters* param);
    void _CleanUp();
    virtual void _LCM_PublishData();

    StaccatoeLocomotionCtrlData<T>* _input_data;

    Task<T>* _centroid_mom_task;
    Task<T>* _body_ori_task;
    Task<T>* _jpos_task;
    Task<T>* _body_pos_task;
    Task<T>* _cam_task;
    Task<T>* _foot_task;

    constexpr static size_t _num_contact = staccatoe_contact::num_foot_contact;
    ContactSpec<T>* _foot_contact[_num_contact];

    //Vec3<T> _Fr_result[humanoid::num_leg];
    Quat<T> _quat_des;
    DVec<T> _centroid_mom_pos_des = DVec<T>::Zero(7);

    Vec3<T> _Kp_body = {250., 250., 100.};
    Vec3<T> _Kd_body = {5.0, 5.0, 1.3};

    Vec3<T> _Kp_ori = {150, 150, 0};
    Vec3<T> _Kd_ori = {2.5, 2.5, 0};

    Vec3<T> _Kp_cam = {0, 0, 0};
    Vec3<T> _Kd_cam = {5, 5, 5};

    Vec3<T> _Kp_clm = {100, 100, 200};
    Vec3<T> _Kd_clm = {2., 2., 5.};
};

#endif

