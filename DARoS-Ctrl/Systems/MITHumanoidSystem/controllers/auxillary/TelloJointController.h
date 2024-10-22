#ifndef Tello_Joint_Controller_H
#define Tello_Joint_Controller_H

#include "JointControl.h"
#include <robots/Tello.h>
#include <tello_joint_data_lcmt.hpp>
#include <tello_joint_command_lcmt.hpp>
#include <Utilities/pretty_print.h>

template <typename T>
class TelloJointController {
 public:
  TelloJointController(const Tello<T> & tello):_tello(tello) {
    for(size_t i(0); i<tello::num_leg; ++i){
      _datas[i] = new JointControlData<T>(tello::num_leg_joint);
      _commands[i] = new JointControlCommand<T>(tello::num_leg_joint);
    }

    for(size_t i(tello::num_leg ); i<tello::num_joint_group; ++i){
      _datas[i] = new JointControlData<T>(tello::num_arm_joint);
      _commands[i] = new JointControlCommand<T>(tello::num_arm_joint);
    }
  }
  ~TelloJointController(){
    for(size_t i(0); i<tello::num_joint_group; ++i) delete _datas[i];
    for(size_t i(0); i<tello::num_joint_group; ++i) delete _commands[i];
  }

  void zeroCommand();
  void updateData(const tello_joint_data_lcmt* data);
  void updateCommand(tello_joint_command_lcmt* command);
  void setEnabled(bool enabled) { _JointsEnabled = enabled; };
  void setLcm(tello_joint_data_lcmt* data, tello_joint_command_lcmt* command);

  // Leg x2, Arm x2, Torso Yaw
  JointControlCommand<T>* _commands[tello::num_joint_group];
  JointControlData<T>* _datas[tello::num_joint_group];

  const Tello<T>& _tello;
  bool _JointsEnabled = false;
  bool _zeroEncoders = false;
  u32 _calibrateEncoders = 0;

  // T _V = _tello._batteryV;
  // T _gr[tello::num_act_joint] = 
  // {_tello._GearRatio, _tello._GearRatio, _tello._GearRatio, _tello._GearRatio, _tello._GearRatio};
  // T _kt[5] = {_tello._smallMotorKT, _tello._smallMotorKT, _tello._largeMotorKT, _tello._largeMotorKT, _tello._smallMotorKT};
  // T _R[5] = {_tello._smallMotorR, _tello._smallMotorR, _tello._largeMotorR, _tello._largeMotorR, _tello._smallMotorR};
  // T _tauMax[5] = {_tello._smallMotorTauMax, _tello._smallMotorTauMax, _tello._largeMotorTauMax, _tello._largeMotorTauMax, _tello._smallMotorTauMax};
};

#endif
