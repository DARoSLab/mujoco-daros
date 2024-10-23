#ifndef Prestoe_STAND_CONTROLLER
#define Prestoe_STAND_CONTROLLER

#include <WBC_Prestoe/WBC_Ctrl.hpp>
#include <PrestoeFBModel.h>

template<typename T>
class PrestoeStandCtrlData{
  public:
    Vec3<T> pBody_des;
    Vec3<T> vBody_des;
    Vec3<T> aBody_des;
    Vec3<T> pBody_RPY_des;
    Vec3<T> vBody_Ori_des;

    Vec3<T> Fr_des[prestoe_contact::num_foot_contact];
    DVec<T> jpos_des = DVec<T>::Zero(prestoe::num_act_joint);
    Vec5<T> contact_state;
};

template<typename T>
class PrestoeStandCtrl: public WBC_Ctrl<T>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PrestoeStandCtrl(const FloatingBaseModel<T> * model);
    virtual ~PrestoeStandCtrl();

  protected:
    virtual void _ContactTaskUpdate(void * input);
    void _ParameterSetup();
    void _CleanUp();
    virtual void _LCM_PublishData();

    PrestoeStandCtrlData<T>* _input_data;

    Task<T>* _body_ori_task;
    Task<T>* _jpos_task;
    Task<T>* _body_pos_task;
    constexpr static size_t _num_contact = prestoe_contact::num_foot_contact;
    ContactSpec<T>* _foot_contact[_num_contact];

    //Vec3<T> _Fr_result[humanoid::num_leg];
    Quat<T> _quat_des;
    DVec<T> _centroid_mom_pos_des = DVec<T>::Zero(7);

    Vec3<T> _Kp_body = {150., 200., 80.};
    Vec3<T> _Kd_body = {1.3, 2.0, 1.3};

    Vec3<T> _Kp_ori = {150, 150, 10};
    Vec3<T> _Kd_ori = {2.5, 1.5, 0.5};

    Vec3<T> _Kp_cam = {0, 0, 0};
    Vec3<T> _Kd_cam = {5, 5, 5};

    Vec3<T> _Kp_clm = {100, 100, 200};
    Vec3<T> _Kd_clm = {2., 2., 5.};
};

#endif

