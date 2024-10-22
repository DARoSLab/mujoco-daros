#ifndef Tello_Loco_CONTROLLER
#define Tello_Loco_CONTROLLER

#include <wbc_ctrl/WBC_Ctrl.hpp>

template<typename T>
class TelloLocoCtrlData{
  public:
    Vec3<T> pBody_des;
    Vec3<T> vBody_des;
    Vec3<T> aBody_des;
    Vec3<T> pBody_RPY_des;
    Vec3<T> vBody_Ori_des;

    Vec3<T> Fr_des[tello_contact::num_foot_contact];
    //Vec4<T> Fr_des[humanoid::num_leg];
    Vec3<T> pFoot_des[tello_contact::num_foot_contact];
    Vec3<T> vFoot_des[tello_contact::num_foot_contact];
    Vec3<T> aFoot_des[tello_contact::num_foot_contact];

    DVec<T> jpos_des = DVec<T>::Zero(tello::num_act_joint);
    Vec4<T> contact_state;
};

template<typename T>
class TelloLocoCtrl: public WBC_Ctrl<T>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TelloLocoCtrl(const FloatingBaseModel<T> * model, const ControlFSMData_Tello<T>* _controlFSMData);
    virtual ~TelloLocoCtrl();

  protected:
    virtual void _ContactTaskUpdate(void * input, ControlFSMData_Tello<T> & data);
    void _ParameterSetup(const TelloParameters* param);
    void _CleanUp();
    virtual void _LCM_PublishData();

    TelloLocoCtrlData<T>* _input_data;

    Task<T>* _body_ori_task;
    Task<T>* _body_pos_task;
    Task<T>* _jpos_task;
    Task<T>* _foot_task[tello_contact::num_foot_contact];

    constexpr static size_t _num_contact = tello_contact::num_foot_contact;
    ContactSpec<T>* _foot_contact[_num_contact];

    //Vec3<T> _Fr_result[humanoid::num_leg];
    Quat<T> _quat_des;
    DVec<T> _centroid_mom_pos_des = DVec<T>::Zero(7);


    Vec3<T> _Kp_clm = {100, 100, 200};
    Vec3<T> _Kd_clm = {2., 2., 5.};
};

#endif

