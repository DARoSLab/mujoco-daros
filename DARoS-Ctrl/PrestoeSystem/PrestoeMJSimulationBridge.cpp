#include "PrestoeMJSimulationBridge.hpp"
#include <Configuration.h>
#include <PrestoeDefinition.h>
#include <PrestoeObsManager.hpp>

using namespace std;

PrestoeMJSimulationBridge::PrestoeMJSimulationBridge(System<double> * sys, const std::string & mj_xml_file):
  MujocoSimulationBridge(sys, mj_xml_file)
{
  // run simulation before run controller
  _ctrl_time += _system->getCtrlDt();
  _prestoe_sys = dynamic_cast<PrestoeSystem<double>*>(sys);
  _prestoe_sys->_obs_manager = new PrestoeObsManager<double>(_mjData);
  _prestoe_sys->_state_ctrl = new StateMachineCtrl<double>(_prestoe_sys->_obs_manager);
  printf("[Prestoe Mujoco Simulation Bridge] Constructed\n");
}


void PrestoeMJSimulationBridge::_UpdateSystemObserver(){
  _prestoe_sys->_obs_manager->UpdateObservers();
}

void PrestoeMJSimulationBridge::_UpdateControlCommand(){
  for(size_t i(0); i<prestoe::num_act_joint; i++)
  {
    _mjData->ctrl[i] = 0.;
  }
}

void PrestoeMJSimulationBridge::_UpdateSystemVisualInfo(){
  // _system->_vis_manager.updateVisual();
}

// void PrestoeMJSimulationBridge::setInitKeyframe(){
//     int keyframe_id = 0;  // Modify the first keyframe

//     mj_resetDataKeyframe(_mjModel, _mjData, keyframe_id);
// }

// void PrestoeMJSimulationBridge::control_callback(const mjModel* m, mjData* d){
//   // cout<<"control_callback"<<endl;
//   if(instance->_iter<1){
//    instance->jtau_cmd_pre.setZero(prestoe::num_act_joint);
//   }
//   instance->jtau_cmd.setZero(prestoe::num_act_joint);
//   if (instance)
//     {
//       // instance->_joint_controller->control_step(instance->jpos_cmd, instance->jvel_cmd, instance->jtau_cmd, instance->Kp, instance->Kd);
//         if(instance->_ctrl_time<=instance->_sim_time){
//               instance->_wbc_controller->control_step(instance->jpos_cmd, instance->jvel_cmd, instance->jtau_cmd, instance->Kp, instance->Kd);
//               instance->jtau_cmd_pre=instance->jtau_cmd;
//         }
//         else{
//               instance->jtau_cmd=instance->jtau_cmd_pre;
//         }
//         for (size_t i(0); i < prestoe::num_act_joint; ++i)
//           {
//             d->ctrl[i]=instance->jtau_cmd[i];
//             // d->ctrl[i] = instance->computeLowLevelCMD(d->qpos[7 + i], d->qvel[7 + i], static_cast<mjtNum>(instance->jpos_cmd[i]),
//             //                              static_cast<mjtNum>(instance->jvel_cmd[i]), static_cast<mjtNum>(instance->jtau_cmd[i]), static_cast<mjtNum>(instance->Kp[i]),
//             //                              static_cast<mjtNum>(instance->Kd[i]));
//             // cout << "u: " << d->ctrl[i] << "\tq: " << d->qpos[7 + i] << "\tqdes: " << instance->jpos_cmd[i] << "\tqd_des: " <<instance->jvel_cmd[i]<<"\tKp: " << instance->Kp[i] << "\tKd: " << instance->Kd[i] << endl;
//           }
//           // exit(0);
//     }
//     instance->LCMPublishData();
// }

// void PrestoeMJSimulationBridge::updateIMUData(IMU_Data& _imudat){
//   int acc_sensor_id = mj_name2id(_model, mjOBJ_SENSOR, "accelerometer");
//   int gyro_sensor_id = mj_name2id(_model, mjOBJ_SENSOR, "gyro");

//   if (acc_sensor_id != -1) {
//       // Access accelerometer data
//       const double* acc_data = _mjDat->sensordata + _model->sensor_adr[acc_sensor_id];
//       // std::cout << "Accelerometer data: " 
//       //           << acc_data[0] << ", " 
//       //           << acc_data[1] << ", " 
//       //           << acc_data[2] << std::endl;
//       for(int i(0);i<3;i++)
//         _imudat.accelerometer[i]=acc_data[i];

//   }

//   if (gyro_sensor_id != -1) {
//       // Access gyro data
//       const double* gyro_data = _mjDat->sensordata + _model->sensor_adr[gyro_sensor_id];
//       // std::cout << "Gyro data: " 
//       //           << gyro_data[0] << ", " 
//       //           << gyro_data[1] << ", " 
//       //           << gyro_data[2] << std::endl;
//       for(int i(0);i<3;i++)
//         _imudat.accelerometer[i]=gyro_data[i];
//   }
// }


// Eigen::VectorXd PrestoeMJSimulationBridge::getCurrentJointPosition(){
//   Eigen::VectorXd current_joint_position(prestoe::num_act_joint);
//   for(int i(0);i<prestoe::num_act_joint;i++){
//     current_joint_position[i]=_mjDat->qpos[7+i];
//   }
//   return current_joint_position;
// }

// Eigen::VectorXd PrestoeMJSimulationBridge::getCurrentBodyPosture(){
//   Eigen::VectorXd current_body_posture(7);
//   for(int i(0);i<7;i++){
//     current_body_posture[i]=_mjDat->qpos[i];
//   }
//   return current_body_posture;
// }
