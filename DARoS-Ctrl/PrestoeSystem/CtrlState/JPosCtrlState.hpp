#ifndef __JPOS_CTRL_STATE_H__
#define __JPOS_CTRL_STATE_H__

#include "State.hpp"
#include <PrestoeObsManager.hpp>

template <typename T>
class JPosCtrlState : public State<T> {
  public:
    JPosCtrlState(ObserverManager<T>* obs_manager);
    virtual ~JPosCtrlState(){}

    virtual void Initialize();
    virtual void OnEnter(); 
    virtual void RunNominal();

  protected:
    ObserverManager<T>* _obs_manager;
    DVec<T> _jpos_ini;
    DVec<T> _jpos_default;
    DVec<T> _swing_freq;
    DVec<T> _swing_amp;

    T _state_time = 0.0;
};

#endif