#ifndef STACCATOE_MJ_SIMULATION_BRIDGE_H
#define STACCATOE_MJ_SIMULATION_BRIDGE_H

#include <MujocoSimulationBridge.hpp>
#include <unistd.h>
#include <iostream>
#include <cppTypes.h>
#include "../Utils/utilities.hpp"
#include "staccatoe.h"

#include "JointPDPosture.hpp"
#include "StaccatoeWBC.hpp"

#include <lcm/lcm-cpp.hpp>
#include "staccatoe_joint_data_lcmt.hpp"
#include "staccatoe_joint_command_lcmt.hpp"
#include "staccatoe_wbc_lcmt.hpp"

class StaccatoeMJSimulationBridge: public MujocoSimulationBridge{
  public:
    StaccatoeMJSimulationBridge(const std::string  mj_xml_file, Controller *controller, const std::string cont_type);
    virtual ~StaccatoeMJSimulationBridge(){
    }

    Eigen::Vector4d getBodyOrientation();

  protected:
    double _sim_time = 0.;
    double _ctrl_time = 0.;
    double _sim_dt = 0.001;
    double _ctrl_dt = 0.002;
    double _mu=0.8;

    lcm::LCM control_lcm;

    static StaccatoeMJSimulationBridge* instance;
    IMU_Data _imu_data;
    Controller* _controller;
    
    JointPDPosture* _joint_controller;
    StaccatoeWBC* _wbc_controller;

    Eigen::VectorXd jpos_cmd;
    Eigen::VectorXd jvel_cmd;
    Eigen::VectorXd jtau_cmd;
    Eigen::VectorXd jtau_cmd_pre;
    Eigen::VectorXd Kp;
    Eigen::VectorXd Kd; 

    virtual void _onestep_simulation();
    void fixTorso();
    void setFixTorsoParams();
    void setInitKeyframe();
    
    template <typename T>
    T computeLowLevelCMD (T q, T qd, T _q_des, T _qd_des, T _torque_ff,
    T _Kp, T _Kd) const {
        return (_Kp*(_q_des - q) - _Kd*(_qd_des - qd) + _torque_ff);
    }

    int _iter=0;
    
    int freejoint_qpos_addr;
    int freejoint_qvel_addr;
    static void control_callback(const mjModel* m,mjData* d);
    void updateIMUData(IMU_Data&);
    Eigen::VectorXd getCurrentJointPosition();
    Eigen::VectorXd getCurrentBodyPosture();
    void initJointController();
    void initWBCController();

    void LCMPublishData();
  
};

#endif
