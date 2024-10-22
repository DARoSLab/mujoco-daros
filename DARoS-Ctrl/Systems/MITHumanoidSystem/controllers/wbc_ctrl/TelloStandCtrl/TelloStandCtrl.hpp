#ifndef Tello_STAND_CONTROLLER
#define Tello_STAND_CONTROLLER

#include <wbc_ctrl/WBC_Ctrl.hpp>

template<typename T>
class TelloStandCtrlData{
  public:
    Vec3<T> pBody_des;
    Vec3<T> vBody_des;
    Vec3<T> aBody_des;
    Vec3<T> pBody_RPY_des;
    Vec3<T> vBody_Ori_des;

    Vec3<T> Fr_des[tello_contact::num_foot_contact];
    //Vec4<T> Fr_des[humanoid::num_leg];

    DVec<T> jpos_des = DVec<T>::Zero(tello::num_act_joint);

};

template<typename T>
class TelloStandCtrl: public WBC_Ctrl<T>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TelloStandCtrl(const FloatingBaseModel<T> * model);
    virtual ~TelloStandCtrl();

  protected:
    virtual void _ContactTaskUpdate(void * input, ControlFSMData_Tello<T> & data);
    void _ParameterSetup(const TelloParameters* param);
    void _CleanUp();
    virtual void _LCM_PublishData();

    TelloStandCtrlData<T>* _input_data;

    Task<T>* _centroid_mom_task;
    Task<T>* _body_ori_task;
    Task<T>* _jpos_task;
    Task<T>* _body_pos_task;
    Task<T>* _cam_task;

    constexpr static size_t _num_contact = tello_contact::num_foot_contact;
    ContactSpec<T>* _foot_contact[_num_contact];

    //Vec3<T> _Fr_result[humanoid::num_leg];
    Quat<T> _quat_des;
    DVec<T> _centroid_mom_pos_des = DVec<T>::Zero(7);

    Vec3<T> _Kp_body = {100., 100., 100.};
    Vec3<T> _Kd_body = {3, 3, 3};

    Vec3<T> _Kp_ori = {120, 120, 120};
    Vec3<T> _Kd_ori = {3, 3, 3};

    Vec3<T> _Kp_cam = {0, 0, 0};
    Vec3<T> _Kd_cam = {5, 5, 5};

    Vec3<T> _Kp_clm = {100, 100, 200};
    Vec3<T> _Kd_clm = {2., 2., 5.};
};

#endif

