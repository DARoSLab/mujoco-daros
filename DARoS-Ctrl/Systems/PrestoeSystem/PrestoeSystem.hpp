#ifndef __PRESTOE_SYSTEM_H__
#define __PRESTOE_SYSTEM_H__

#include <System.hpp>
#include <StateMachineCtrl.hpp>
#include <PrestoeObsManager.hpp>

template<typename T>
class PrestoeSystem: public System<T>{
  public:
    PrestoeSystem(const std::string & setup_file);
    virtual ~PrestoeSystem(){ }

    bool Initialization();
    virtual void runCtrl();

    // Managers and StateMachineCtrl are constructed in Bridge and passed to System
    ObserverManager<T>* _obs_manager;
    // UserInputManager<T>* _user_input_manager;
    // VisualManager<T>* _vis_manager;

    StateMachineCtrl<T>* _state_ctrl;
    StateList _test_state_id;

  protected:
    void _ReadConfig(const std::string & file_name);
    bool _initial_run = true;
};

#endif