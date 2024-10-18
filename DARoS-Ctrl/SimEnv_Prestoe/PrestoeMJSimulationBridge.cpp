#include "PrestoeMJSimulationBridge.hpp"
#include "OrientationTools.hpp"
#include "MatrixPrint.h"

using namespace std;

PrestoeMJSimulationBridge* PrestoeMJSimulationBridge::instance = nullptr;

PrestoeMJSimulationBridge::PrestoeMJSimulationBridge(
  const std::string mj_xml_file, Controller* controller, const std::string cont_type) : MujocoSimulationBridge(mj_xml_file), control_lcm(getLcmUrl(255))  
{
  // _controller=controller;
 if(cont_type=="wbc"){
    _wbc_controller=dynamic_cast<PrestoeWBC*>(controller);
    _wbc_controller->setModelParams(_model,_mjDat,_mu,10);
    setInitKeyframe();
    initWBCController();
  }else if(cont_type=="joint"){
    _joint_controller=dynamic_cast<JointPDPosture*>(controller);
    setInitKeyframe();
    initJointController();    
  }

  instance = this;
  mjcb_control=control_callback;
  if (_ctrl_dt < _sim_dt)
  {
    printf("[Error] Controller runs faster than Simulation\n");
    exit(0);
  }
  // run simulation before run controller
  _ctrl_time += _ctrl_dt;
  // setFixTorsoParams();

  jpos_cmd.Zero(prestoe::num_act_joint);
  jvel_cmd.Zero(prestoe::num_act_joint);
  jtau_cmd.Zero(prestoe::num_act_joint);
  jtau_cmd_pre.Zero(prestoe::num_act_joint);
  Kp.Zero(prestoe::num_act_joint);
  Kd.Zero(prestoe::num_act_joint);

  printf("[Prestoe Mujoco Simulation Bridge] Constructed\n");
}

void PrestoeMJSimulationBridge::_onestep_simulation()
{
  
  while (_sim_time < _ctrl_time)
  {
    _sim_time += _sim_dt;
    mj_step(_model, _mjDat);
    // _mjDat->time += _sim_dt;
    // fixTorso();
  }
  updateIMUData(_imu_data);


  _iter++;
  _ctrl_time+=_ctrl_dt;
}

void PrestoeMJSimulationBridge::setInitPose()
{

  // NOTE: This is written for floating base models
  mjtNum *qpos_init = _model->qpos0;
  mjtNum *qvel_init = _mjDat->qvel;
  mjtNum *qdata_init = _mjDat->qpos;

  // Set body position and orientation
  qpos_init[0] = 0;
  qpos_init[1] = 0;
  qpos_init[2] = 2;

  qdata_init[0] = 0;
  qdata_init[1] = 0;
  qdata_init[2] = 2;

  // cout<<"body pos: "<<qdata_init[0]<<", "<<qdata_init[1]<<", "<<qdata_init[2]<<endl;

  qpos_init[3] = 1;
  qpos_init[4] = 0;
  qpos_init[5] = 0;
  qpos_init[6] = 0;
  qdata_init[3] = 1;
  qdata_init[4] = 0;
  qdata_init[5] = 0;
  qdata_init[6] = 0;

  // Set the joint angles from state.q (remaining nq-7 values)
  // cout<<"q: "<<endl;
  for (int i = 0; i < prestoe::num_act_joint; ++i)
  {
    qpos_init[7 + i] = __init_jpos[i];
    qdata_init[7 + i] = __init_jpos[i];
    // cout<<qdata_init[7+i]<<endl;
  }

  // Set body velocity
  qvel_init[0] = 0;
  qvel_init[1] = 0;
  qvel_init[2] = 0;

  // Set joint velocities
  for (int i = 0; i < 23; ++i)
  {
    qvel_init[7 + i] = 0;
  }
}

void PrestoeMJSimulationBridge::setFixTorsoParams()
{
  // Set position (x, y, z)
  _mjDat->qpos[freejoint_qpos_addr + 0] = 0;
  _mjDat->qpos[freejoint_qpos_addr + 1] = 0;
  _mjDat->qpos[freejoint_qpos_addr + 2] = 2;

  // Set orientation (quaternion w, x, y, z)
  _mjDat->qpos[freejoint_qpos_addr + 3] = 1;
  _mjDat->qpos[freejoint_qpos_addr + 4] = 0;
  _mjDat->qpos[freejoint_qpos_addr + 5] = 0;
  _mjDat->qpos[freejoint_qpos_addr + 6] = 0;

  // Set linear velocity (vx, vy, vz) to zero
  _mjDat->qvel[freejoint_qvel_addr + 0] = 0;
  _mjDat->qvel[freejoint_qvel_addr + 1] = 0;
  _mjDat->qvel[freejoint_qvel_addr + 2] = 0;

  // Set angular velocity (wx, wy, wz) to zero
  _mjDat->qvel[freejoint_qvel_addr + 3] = 0;
  _mjDat->qvel[freejoint_qvel_addr + 4] = 0;
  _mjDat->qvel[freejoint_qvel_addr + 5] = 0;
}

void PrestoeMJSimulationBridge::fixTorso()
{
  _mjDat->qvel[freejoint_qvel_addr + 0] = 0;
  _mjDat->qvel[freejoint_qvel_addr + 1] = 0;
  _mjDat->qvel[freejoint_qvel_addr + 2] = 0;
  _mjDat->qvel[freejoint_qvel_addr + 3] = 0;
  _mjDat->qvel[freejoint_qvel_addr + 4] = 0;
  _mjDat->qvel[freejoint_qvel_addr + 5] = 0;
}

void PrestoeMJSimulationBridge::setInitKeyframe(){
    int keyframe_id = 0;  // Modify the first keyframe

    mj_resetDataKeyframe(_model, _mjDat, keyframe_id);
}

void PrestoeMJSimulationBridge::control_callback(const mjModel* m, mjData* d){
  // cout<<"control_callback"<<endl;
  if(instance->_iter<1){
   instance->jtau_cmd_pre.setZero(prestoe::num_act_joint);
  }
  instance->jtau_cmd.setZero(prestoe::num_act_joint);
  if (instance)
    {
      // instance->_joint_controller->control_step(instance->jpos_cmd, instance->jvel_cmd, instance->jtau_cmd, instance->Kp, instance->Kd);
        if(instance->_ctrl_time<=instance->_sim_time){
              instance->_wbc_controller->control_step(instance->jpos_cmd, instance->jvel_cmd, instance->jtau_cmd, instance->Kp, instance->Kd);
              instance->jtau_cmd_pre=instance->jtau_cmd;
        }
        else{
              instance->jtau_cmd=instance->jtau_cmd_pre;
        }
        for (size_t i(0); i < prestoe::num_act_joint; ++i)
          {
            d->ctrl[i]=instance->jtau_cmd[i];
            // d->ctrl[i] = instance->computeLowLevelCMD(d->qpos[7 + i], d->qvel[7 + i], static_cast<mjtNum>(instance->jpos_cmd[i]),
            //                              static_cast<mjtNum>(instance->jvel_cmd[i]), static_cast<mjtNum>(instance->jtau_cmd[i]), static_cast<mjtNum>(instance->Kp[i]),
            //                              static_cast<mjtNum>(instance->Kd[i]));
            // cout << "u: " << d->ctrl[i] << "\tq: " << d->qpos[7 + i] << "\tqdes: " << instance->jpos_cmd[i] << "\tqd_des: " <<instance->jvel_cmd[i]<<"\tKp: " << instance->Kp[i] << "\tKd: " << instance->Kd[i] << endl;
          }
          // exit(0);
    }
    instance->LCMPublishData();
}

void PrestoeMJSimulationBridge::updateIMUData(IMU_Data& _imudat){
  int acc_sensor_id = mj_name2id(_model, mjOBJ_SENSOR, "accelerometer");
  int gyro_sensor_id = mj_name2id(_model, mjOBJ_SENSOR, "gyro");

  if (acc_sensor_id != -1) {
      // Access accelerometer data
      const double* acc_data = _mjDat->sensordata + _model->sensor_adr[acc_sensor_id];
      // std::cout << "Accelerometer data: " 
      //           << acc_data[0] << ", " 
      //           << acc_data[1] << ", " 
      //           << acc_data[2] << std::endl;
      for(int i(0);i<3;i++)
        _imudat.accelerometer[i]=acc_data[i];

  }

  if (gyro_sensor_id != -1) {
      // Access gyro data
      const double* gyro_data = _mjDat->sensordata + _model->sensor_adr[gyro_sensor_id];
      // std::cout << "Gyro data: " 
      //           << gyro_data[0] << ", " 
      //           << gyro_data[1] << ", " 
      //           << gyro_data[2] << std::endl;
      for(int i(0);i<3;i++)
        _imudat.accelerometer[i]=gyro_data[i];
  }
}


Eigen::VectorXd PrestoeMJSimulationBridge::getCurrentJointPosition(){
  Eigen::VectorXd current_joint_position(prestoe::num_act_joint);
  for(int i(0);i<prestoe::num_act_joint;i++){
    current_joint_position[i]=_mjDat->qpos[7+i];
  }
  return current_joint_position;
}

Eigen::VectorXd PrestoeMJSimulationBridge::getCurrentBodyPosture(){
  Eigen::VectorXd current_body_posture(7);
  for(int i(0);i<7;i++){
    current_body_posture[i]=_mjDat->qpos[i];
  }
  return current_body_posture;
}

//initialize joint controller 
void PrestoeMJSimulationBridge::initJointController(){
  Eigen::VectorXd targetPos(prestoe::num_act_joint);
  Eigen::VectorXd curr_jpos=getCurrentJointPosition();

  double finalPos[prestoe::num_act_joint]={0, 0, 0, -1.0472, 1.39626, -0.7417649, 0, 0, 0, 0, -1.0472, 1.39626, -0.7417649,  0, 0, 0, 0, 0, 0,0, 0, 0, 0};
  for(int i(0);i<prestoe::num_act_joint;i++){
    targetPos[i]=finalPos[i];
  }
  // _joint_controller->computeJointTrajectory(curr_jpos,targetPos,1000);
    _joint_controller->computeJointTrajectoryBZC(curr_jpos,targetPos,1000);

  Eigen::VectorXd _kp_joint(prestoe::num_act_joint);
  Eigen::VectorXd _kd_joint(prestoe::num_act_joint); 

  _kp_joint<<100,   100, 100, 200, 100, 150, 100, 100,  100, 100, 200, 100, 150, 100, 100,  100, 100, 20, 20, 100, 100, 20, 20;
  _kd_joint<<5e-2,  1e-2, 5e-2, 1e-2, 1e-2, 1e-3, 1e-2, 1e-2,   1e-2, 5e-2, 1e-2, 1e-2, 1e-3, 1e-2, 1e-2,   5e-2, 5e-2, 1e-2, 1e-2,5e-2, 5e-2, 1e-2, 1e-2;
  _joint_controller->setGains(_kp_joint,_kd_joint);
}

void PrestoeMJSimulationBridge::initWBCController(){
  Eigen::VectorXd targetPosture=getCurrentBodyPosture();
  Eigen::VectorXd vbody_des=Eigen::VectorXd::Zero(6);
  Eigen::Vector3d acc_des=Eigen::Vector3d::Zero();
  Eigen::Vector4d bodyquat_des=targetPosture.tail(4);
  Eigen::Vector3d bodyRPY_des=orientation_tools::quaternionToRPY(bodyquat_des);
  
  targetPosture[2]=0.9;
  bodyRPY_des[1]=0.4;
  _wbc_controller->setFBTargets(targetPosture.head(3),bodyRPY_des,vbody_des,acc_des);
  Eigen::VectorXd _kp(6);
  Eigen::VectorXd _kd(6); 

  // _kp<<1, 1, 1, 1, 1, 1;
  // _kd<<1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3;
  _kp<<1, 1, 10, 1, 10, 1;
  _kd<<1e-3, 1e-3, 10e-3, 1e-3, 10e-3, 1e-3;
  _wbc_controller->setGains(_kp,_kd);
}

void PrestoeMJSimulationBridge::LCMPublishData()
{
 
  prestoe_joint_data_lcmt jdata;
  prestoe_joint_command_lcmt jcmd;
  prestoe_wbc_lcmt wbcdata;


  // for (int i = 0; i < staccatoe::num_act_joint; i++)
  // {
  //   jdata.q[i]=_mjDat->qpos[7+i];
  //   jdata.qd[i]=_mjDat->qvel[7+i];
  //   // jdata.tau_est[i]=_mjDat->qfrc_applied[7+i];

  //   jcmd.q_des[i]=jpos_cmd[i];
  //   jcmd.qd_des[i]=jvel_cmd[i];
  //   jcmd.tau_ff[i]=jtau_cmd[i];

  //   wbcdata.tau_ff[i]=jtau_cmd[i];

  // }
  // Eigen::Vector4d body_quat=getBodyOrientation();
    // Eigen::Vector4d body_quat(_mjDat->qpos[3],_mjDat->qpos[4],_mjDat->qpos[5],_mjDat->qpos[6]);

 
  Eigen::Vector3d bodyRPY=orientation_tools::quaternionToRPY(Eigen::Vector4d(_mjDat->qpos[3],_mjDat->qpos[4],_mjDat->qpos[5],_mjDat->qpos[6]));
  
 
  for (int i = 0; i < 3; i++)
  {
    wbcdata.pBody_des[i]=_wbc_controller->_pbody_des[i];
    wbcdata.bodyRPY_des[i]=_wbc_controller->_bodyRPY_des[i];
    wbcdata.pBody[i]=_mjDat->qpos[i];
    wbcdata.bodyRPY[i]=bodyRPY[i];
  }

  for (int i = 0; i < 6; i++)
  {
    wbcdata.vBody[i]=_mjDat->qvel[i];
    wbcdata.vBody_des[i]=_wbc_controller->_vbody_des[i];
  }

  // for (int i = 0; i < 15; i++)
  // {
  //   wbcdata.Fr_des[i]=_wbc_controller->Fr_des[i];
  // }
  control_lcm.publish("prestoe_joint_data", &jdata);
  control_lcm.publish("prestoe_joint_cmd", &jcmd);
  control_lcm.publish("prestoe_wbc_data", &wbcdata);
}

// Eigen::Vector4d PrestoeMJSimulationBridge::getBodyOrientation(){
//   Eigen::Vector4d bodyquat;
//   for(int i(0);i<4;i++){
//     bodyquat[i]=_mjDat->qpos[3+i];
//   }
//   return bodyquat;
// }