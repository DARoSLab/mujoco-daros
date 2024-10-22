#ifndef __JPOS_CTRL_STATE_H__
#define __JPOS_CTRL_STATE_H__

#include "State.hpp"

template <typename T> class ObserverManager;
template <typename T> class Command;

template <typename T>
class JPosCtrlState : public State<T> {
  public:
    JPosCtrlState(ObserverManager<T>* obs_manager, PrestoeSystem<T>* prestoe_system);
    virtual ~JPosCtrlState(){}

    virtual void Initialize();
    virtual void OnEnter(); 
    virtual void RunNominal();
    virtual Command<T>* GetCommand() { return _jtorque_cmd; }

  protected:
    void _ReadConfig(const std::string & file_name);
    ObserverManager<T>* _obs_manager;
    DVec<T> _jpos_ini;
    DVec<T> _jpos_default;
    DVec<T> _swing_freq;
    DVec<T> _swing_amp;
    DVec<T> _Kp, _Kd;

    Command<T>* _jtorque_cmd;
};

#endif