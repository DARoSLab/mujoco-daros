#ifndef Prestoe_STAND_CONTROLLER
#define Prestoe_STAND_CONTROLLER

#include <WBC_Prestoe/WBC_Ctrl.hpp>
#include <PrestoeFBModel.h>

template<typename T>
class PrestoeBoxPickupCtrlData{
  public:
    Vec3<T> pCoM_des;
    Vec3<T> pBody_RPY_des;

    Vec3<T> rHand_pos_des;
    Vec3<T> lHand_pos_des;

    Vec3<T> Fr_des[prestoe_contact::num_foot_contact];
    DVec<T> jpos_des = DVec<T>::Zero(prestoe::num_act_joint);
};

template<typename T>
class PrestoeBoxPickupCtrl: public WBC_Ctrl<T>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PrestoeBoxPickupCtrl(const FloatingBaseModel<T> * model, const std::string & config_file);
    virtual ~PrestoeBoxPickupCtrl();

  protected:
    virtual void _ContactTaskUpdate(void * input);
    void _CleanUp();
    virtual void _LCM_PublishData();

    void _ReadConfig(const std::string & config_file);

    PrestoeBoxPickupCtrlData<T>* _input_data;

    Task<T>* _body_ori_task;
    Task<T>* _jpos_task;
    Task<T>* _com_task;
    Task<T>* _rhand_task;
    Task<T>* _lhand_task;

    constexpr static size_t _num_contact = prestoe_contact::num_foot_contact;
    ContactSpec<T>* _foot_contact[_num_contact];

    Quat<T> _quat_des;
};

#endif

