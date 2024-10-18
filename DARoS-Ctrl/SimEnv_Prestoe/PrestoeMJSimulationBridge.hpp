#ifndef PRESTOE_MJ_SIMULATION_BRIDGE_H
#define PRESTOE_MJ_SIMULATION_BRIDGE_H

#include <MujocoSimulationBridge.hpp>
#include <unistd.h>
#include <iostream>
#include <cppTypes.h>
#include "../Utils/utilities.hpp"
#include "prestoe.h"

#include "JointPDPosture.hpp"
#include "PrestoeWBC.hpp"

#include <lcm/lcm-cpp.hpp>
#include "prestoe_joint_data_lcmt.hpp"
#include "prestoe_joint_command_lcmt.hpp"
#include "prestoe_wbc_lcmt.hpp"

class PrestoeMJSimulationBridge: public MujocoSimulationBridge{
  public:
    PrestoeMJSimulationBridge(const std::string  mj_xml_file, 
    Controller *controller, const std::string cont_type);
    virtual ~PrestoeMJSimulationBridge(){
    }
 
    // Eigen::Vector4d getBodyOrientation();

  protected:
    double _sim_time = 0.;
    double _ctrl_time = 0.;
    double _sim_dt = 0.001;
    double _ctrl_dt = 0.002;
    double _mu=0.8;

    lcm::LCM control_lcm;

    static PrestoeMJSimulationBridge* instance;
    IMU_Data _imu_data;
    Controller* _controller;

    JointPDPosture* _joint_controller;
    PrestoeWBC* _wbc_controller;

    Eigen::VectorXd jpos_cmd;
    Eigen::VectorXd jvel_cmd;
    Eigen::VectorXd jtau_cmd;
    Eigen::VectorXd jtau_cmd_pre;
    Eigen::VectorXd Kp;
    Eigen::VectorXd Kd;
   

    virtual void _onestep_simulation();
    void setInitPose();
    void fixTorso();
    void setFixTorsoParams();
    void setInitKeyframe();
    
    template <typename T>
    T computeLowLevelCMD (T q, T qd, T _q_des, T _qd_des, T _torque_ff,
    T _Kp, T _Kd) const {
        return (_Kp*(_q_des - q) - _Kd*(_qd_des - qd) + _torque_ff);
    }

    int _iter=0;
    // double _init_jpos[23]={0.0, 0.0, 0.0, -1.02675541724, -1.918911862865, 0.89215611402, 0.0, 0.0,0.0, 0.0, -1.02675541724, -1.918911862865, 0.89215611402,  0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0};
    // double __init_jpos[23]={0.0,
    //             0.0, 0.0, -1.02675541724, 1.918911862865, -0.89215611402, 0.0, 0.0,
    //             0.0, 0.0, -1.02675541724, 1.918911862865, -0.89215611402,  0.0, 0.0,
    //             0.0, 0.0, 0.0, 0.0,
    //             0.0, 0.0, 0.0, 0.0};
    double __init_jpos[23]={0.0,
                0.0, 0.0, -1.88496, 2.0944, -0.872665, 0.0, 0.0,
                0.0, 0.0,-1.88496, 2.0944, -0.872665, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0};
      // double __init_jpos[23]={0.0,
      //           0.0, 0.0, 0, 0, 0, 0.0, 0.0,
      //           0.0, 0.0, 0, 0, 0,  0.0, 0.0,
      //           0.0, 0.0, 0.0, 0.0,
      //           0.0, 0.0, 0.0, 0.0};
    double kp_joint[23]={100, 
                      100, 100, 200, 50, 200, 20, 200,
                      100, 100, 200, 50, 200, 20, 200,
                      100, 100, 20, 20,
                      100, 100, 20, 20};
    //  double kp_joint[23]={0, 
    //                   0, 0, 0, 0, 0, 0, 0,
    //                   0, 0, 0, 0, 0, 0, 0,
    //                   0, 0, 0, 0,
    //                   0, 0, 0, 0};

     double kd_joint[23]={5e-2, 
                      5e-2, 5e-2, 500e-2, 500e-2, 100e-2, 1e-2, 100e-2,
                      5e-2, 5e-2, 500e-2, 500e-2, 100e-2, 1e-2, 100e-2,
                      5e-2, 5e-2, 1e-2, 1e-2,
                      5e-2, 5e-2, 1e-2, 1e-2};

    //  double kd_joint[23]={0, 
    //                   0, 0, 0, 0, 0, 0, 0,
    //                   0, 0, 0, 0, 0, 0, 0,
    //                   0, 0, 0, 0,
    //                   0, 0, 0, 0};

    // DVec<double> __init_jpos = DVec<double>::Zero(23);
    int freejoint_qpos_addr;
    int freejoint_qvel_addr;
    static void control_callback(const mjModel* m,mjData* d);
    void updateIMUData(IMU_Data&);
    Eigen::VectorXd getCurrentJointPosition();
    Eigen::VectorXd getCurrentBodyPosture();
    void initJointController();
    void initWBCController();
    // void control_callback(const mjModel* m, mjData* d) override {
    //     cout<<"i am in the control loop"<<endl;
    // }
    void LCMPublishData(); 
};

#endif
