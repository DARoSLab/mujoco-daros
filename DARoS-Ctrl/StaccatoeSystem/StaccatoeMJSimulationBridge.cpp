#include "StaccatoeMJSimulationBridge.hpp"
#include "OrientationTools.hpp"
#include "MatrixPrint.h"

using namespace std;

StaccatoeMJSimulationBridge* StaccatoeMJSimulationBridge::instance = nullptr;

StaccatoeMJSimulationBridge::StaccatoeMJSimulationBridge(const std::string mj_xml_file, Controller* controller, const std::string cont_type) : MujocoSimulationBridge(mj_xml_file),control_lcm(getLcmUrl(255))
{
  // _controller=controller;
  if(cont_type=="wbc"){
    _wbc_controller=dynamic_cast<StaccatoeWBC*>(controller);
    _wbc_controller->setModelParams(_model,_mjDat,_mu,5);
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

  jpos_cmd.Zero(staccatoe::num_act_joint);
  jvel_cmd.Zero(staccatoe::num_act_joint);
  jtau_cmd.Zero(staccatoe::num_act_joint);
  jtau_cmd_pre.Zero(staccatoe::num_act_joint);
  Kp.Zero(staccatoe::num_act_joint);
  Kd.Zero(staccatoe::num_act_joint);

  printf("[Staccatoe Mujoco Simulation Bridge] Constructed\n");
  
    if (!control_lcm.good()) {
      cout<<"LCM is not good"<<endl;
    }
}

void StaccatoeMJSimulationBridge::_onestep_simulation()
{
  
  while (_sim_time < _ctrl_time)
  {
    _sim_time += _sim_dt;
    mj_step(_model, _mjDat);
    // _mjDat->time += _sim_dt;
    // fixTorso();
  }
  updateIMUData(_imu_data);
  // if (_iter > 2)
  //   exit(0);

  _iter++;
  _ctrl_time+=_ctrl_dt;
}



void StaccatoeMJSimulationBridge::setFixTorsoParams()
{
  // Set position (x, y, z)
  _mjDat->qpos[0] = 0;
  _mjDat->qpos[1] = 0;
  _mjDat->qpos[2] = 0.87;

  // Set orientation (quaternion w, x, y, z)
  _mjDat->qpos[3] = 1;
  _mjDat->qpos[4] = 0;
  _mjDat->qpos[5] = 0;
  _mjDat->qpos[6] = 0;

  // Set linear velocity (vx, vy, vz) to zero
  _mjDat->qvel[0] = 0;
  _mjDat->qvel[1] = 0;
  _mjDat->qvel[2] = 0;

  // Set angular velocity (wx, wy, wz) to zero
  _mjDat->qvel[3] = 0;
  _mjDat->qvel[4] = 0;
  _mjDat->qvel[5] = 0;
}

void StaccatoeMJSimulationBridge::fixTorso()
{
  _mjDat->qvel[0] = 0;
  _mjDat->qvel[1] = 0;
  _mjDat->qvel[2] = 0;
  _mjDat->qvel[3] = 0;
  _mjDat->qvel[4] = 0;
  _mjDat->qvel[5] = 0;
}

void StaccatoeMJSimulationBridge::setInitKeyframe(){
    int keyframe_id = 0;  // Modify the first keyframe

    mj_resetDataKeyframe(_model, _mjDat, keyframe_id);
}

void StaccatoeMJSimulationBridge::control_callback(const mjModel* m, mjData* d){
  // cout<<"control_callback"<<endl;
  if(instance->_iter<1){
    instance->jtau_cmd_pre.setZero(staccatoe::num_act_joint);
  }
  instance->jtau_cmd.setZero(staccatoe::num_act_joint);
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
      //print instance->jtau_cmd
        // printEigen(instance->jtau_cmd,"jtau_cmd");
        // exit(0);
        for (size_t i(0); i < staccatoe::num_act_joint; ++i)
          {
            // d->ctrl[i] = instance->computeLowLevelCMD(d->qpos[7 + i], d->qvel[7 + i], static_cast<mjtNum>(instance->jpos_cmd[i]),
            //                              static_cast<mjtNum>(instance->jvel_cmd[i]), static_cast<mjtNum>(instance->jtau_cmd[i]), static_cast<mjtNum>(instance->Kp[i]),
            //                              static_cast<mjtNum>(instance->Kd[i]));
            
            d->ctrl[i]=instance->jtau_cmd[i];
            // d->ctrl[i] = 0;
            // cout << "u: " << d->ctrl[i] << "\tq: " << d->qpos[7 + i] << "\tqdes: " << instance->jpos_cmd[i] << "\tqd_des: " <<instance->jvel_cmd[i]<<"\tKp: " << instance->Kp[i] << "\tKd: " << instance->Kd[i] << endl;
          }
          // cout << "u: " << d->ctrl[1] << "\tq: " << d->qpos[7 + 1] << "\tqdes: " << instance->jpos_cmd[1] << "\tqd_des: " <<instance->jvel_cmd[1]<<"\tKp: " << instance->Kp[1] << "\tKd: " << instance->Kd[1] << endl;
          // exit(0);
    }
    instance->LCMPublishData();
}

void StaccatoeMJSimulationBridge::updateIMUData(IMU_Data& _imudat){
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


Eigen::VectorXd StaccatoeMJSimulationBridge::getCurrentJointPosition(){
  Eigen::VectorXd current_joint_position(staccatoe::num_act_joint);
  for(int i(0);i<staccatoe::num_act_joint;i++){
    current_joint_position[i]=_mjDat->qpos[7+i];
  }
  return current_joint_position;
}

Eigen::VectorXd StaccatoeMJSimulationBridge::getCurrentBodyPosture(){
  Eigen::VectorXd current_body_posture(7);
  for(int i(0);i<7;i++){
    current_body_posture[i]=_mjDat->qpos[i];
  }
  return current_body_posture;
}

//initialize joint controller
void StaccatoeMJSimulationBridge::initJointController(){
  Eigen::VectorXd targetPos(staccatoe::num_act_joint);
  Eigen::VectorXd curr_jpos=getCurrentJointPosition();

  double finalPos[staccatoe::num_act_joint]={0, -1.0472, 1.39626, -0.7417649, 0, 0};
  for(int i(0);i<staccatoe::num_act_joint;i++){
    targetPos[i]=finalPos[i];
    // targetPos[i]=curr_jpos[i];
    // targetPos[i]=0;
  }
  // _joint_controller->computeJointTrajectory(curr_jpos,targetPos,1000);
    _joint_controller->computeJointTrajectoryBZC(curr_jpos,targetPos,2);

  Eigen::VectorXd _kp_joint(staccatoe::num_act_joint);
  Eigen::VectorXd _kd_joint(staccatoe::num_act_joint); 

  _kp_joint<<100, 100, 200, 100, 150, 100;
  _kd_joint<<1e-2, 5e-2, 5, 1e-2, 1e-3, 1e-2;
  _joint_controller->setGains(_kp_joint,_kd_joint);
}

//initialize WBC controller
void StaccatoeMJSimulationBridge::initWBCController(){
  Eigen::VectorXd targetPosture=getCurrentBodyPosture();
  Eigen::VectorXd vbody_des=Eigen::VectorXd::Zero(6);
  Eigen::Vector3d acc_des=Eigen::Vector3d::Zero();
  Eigen::Vector4d bodyquat_des=targetPosture.tail(4);
  Eigen::Vector3d bodyRPY_des=orientation_tools::quaternionToRPY(bodyquat_des);
  
  targetPosture[2]=0.4;
  // bodyRPY_des[1]=-0.4;
  _wbc_controller->setFBTargets(targetPosture.head(3),bodyRPY_des,vbody_des,acc_des);
  Eigen::VectorXd _kp(6);
  Eigen::VectorXd _kd(6); 

  _kp<<1, 1, 1, 1, 1, 1;
  _kd<<1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3;
  // _kp<<1, 1, 1, 1, 1, 1;
  // _kd<<1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3;
  _wbc_controller->setGains(_kp,_kd);
}

void StaccatoeMJSimulationBridge::LCMPublishData()
{
 
  staccatoe_joint_data_lcmt jdata;
  staccatoe_joint_command_lcmt jcmd;
  staccatoe_wbc_lcmt wbcdata;


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
  Eigen::Vector4d body_quat=getBodyOrientation();
    // Eigen::Vector4d body_quat(_mjDat->qpos[3],_mjDat->qpos[4],_mjDat->qpos[5],_mjDat->qpos[6]);

 
  Eigen::Vector3d bodyRPY=orientation_tools::quaternionToRPY(getBodyOrientation());
  
 
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
  control_lcm.publish("staccatoe_joint_data", &jdata);
  control_lcm.publish("staccatoe_joint_cmd", &jcmd);
  control_lcm.publish("staccatoe_wbc_data", &wbcdata);
}

Eigen::Vector4d StaccatoeMJSimulationBridge::getBodyOrientation(){
  Eigen::Vector4d bodyquat;
  for(int i(0);i<4;i++){
    bodyquat[i]=_mjDat->qpos[3+i];
  }
  return bodyquat;
}