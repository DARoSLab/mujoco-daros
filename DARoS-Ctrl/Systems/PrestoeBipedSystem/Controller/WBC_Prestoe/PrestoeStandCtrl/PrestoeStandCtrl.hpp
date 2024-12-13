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

    PrestoeStandCtrl(const FloatingBaseModel<T> * model, const std::string & config_file);
    virtual ~PrestoeStandCtrl();

  protected:
    virtual void _ContactTaskUpdate(void * input);
    void _CleanUp();
    virtual void _LCM_PublishData();

    void _ReadConfig(const std::string & config_file);

    PrestoeStandCtrlData<T>* _input_data;

    Task<T>* _body_ori_task;
    Task<T>* _jpos_task;
    Task<T>* _body_pos_task;
    constexpr static size_t _num_contact = prestoe_contact::num_foot_contact;
    ContactSpec<T>* _foot_contact[_num_contact];

    //Vec3<T> _Fr_result[humanoid::num_leg];
    Quat<T> _quat_des;
};

#endif

