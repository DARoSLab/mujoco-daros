#ifndef __MINIARM_SYSTEM_H__
#define __MINIARM_SYSTEM_H__

#include <System.hpp>
#include <StateMachineCtrl.hpp>
#include <MiniArmObsManager.hpp>

template<typename T>
class MiniArmSystem: public System<T>{
  public:
    MiniArmSystem(const std::string & setup_file);
    virtual ~MiniArmSystem(){ }

    virtual void runCtrl();

    // Managers and StateMachineCtrl are constructed in Bridge and passed to System
    ObserverManager<T>* _obs_manager;
    StateMachineCtrl<T>* _state_ctrl;

  protected:
    void _ReadConfig(const std::string & file_name);
};

#endif