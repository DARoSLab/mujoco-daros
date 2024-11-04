#include "MiniArmMJSimulationBridge.hpp"
#include <Configuration.h>
#include <MiniArmDefinition.h>
#include <MiniArmObsManager.hpp>
#include <Command.hpp>
#include <State.hpp>
#include <pretty_print.h>

using namespace std;

MiniArmMJSimulationBridge::MiniArmMJSimulationBridge(System<double> * sys, const std::string & config_file):
  MujocoSimulationBridge(sys, config_file)
{
  // run simulation before run controller
  _ctrl_time += _system->getCtrlDt();
  _miniarm_sys = dynamic_cast<MiniArmSystem<double>*>(sys);
  _miniarm_sys->_obs_manager = new MiniArmObsManager<double>(_mjData);
  _miniarm_sys->_state_ctrl = new StateMachineCtrl<double>(_miniarm_sys->_obs_manager, _miniarm_sys);
  printf("[MiniArm Mujoco Simulation Bridge] Constructed\n");
}


void MiniArmMJSimulationBridge::_UpdateSystemObserver(){
  _miniarm_sys->_obs_manager->UpdateObservers();
}

void MiniArmMJSimulationBridge::_UpdateControlCommand(){
  DVec<double> jtorque_output(miniarm::num_act_joint);
  Command<double>* cmd = _miniarm_sys->_state_ctrl->_curr_State->GetCommand();

  DVec<double> jpos(miniarm::num_act_joint);
  DVec<double> jvel(miniarm::num_act_joint);

  for(size_t i(0); i<miniarm::num_act_joint; ++i){
    jpos[i] = static_cast<double>(_mjData->qpos[i]);
    jvel[i] = static_cast<double>(_mjData->qvel[i]);
  }
 
  if(cmd->_type == CommandType::JPOS_CMD){
    dynamic_cast<JPosCommand<double>*>(cmd)->ComputeTorqueCommand(jtorque_output, jpos, jvel);
    pretty_print(jpos, std::cout, "jpos");
    pretty_print(jvel, std::cout, "jvel");

  }else if(cmd->_type == CommandType::JTORQUE_POS_CMD){
    dynamic_cast<JTorquePosCommand<double>*>(cmd)->ComputeTorqueCommand(jtorque_output, jpos, jvel);

  }else{
    printf("[MiniArmMJSimulationBridge] Unknown command type\n");
    exit(0);
  }

  for(size_t i(0); i<miniarm::num_act_joint; ++i)
  {
    _mjData->ctrl[i] = jtorque_output[i];
  }
  std::cout<<"[MiniArmMJSimulationBridge] jtorque_output: "<<jtorque_output.transpose()<<std::endl;
  std::cout<<"[MiniArmMJSimulationBridge] jtorque_output: "<<(*_mjData->ctrl)<<std::endl;
}

void MiniArmMJSimulationBridge::_UpdateSystemVisualInfo(){ }


