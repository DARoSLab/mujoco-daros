#ifndef __CTRL_STATE_H__
#define __CTRL_STATE_H__

#include <cppTypes.h>
#include <Configuration.h>
#include <PrestoeDefinition.h>
#include <PrestoeSystem.hpp>

template<typename T> class Command;

template <typename T> 
class SystemInfo{
public:
    SystemInfo(PrestoeSystem<T>* prestoe_system){
        _ctrl_dt = prestoe_system->getCtrlDt();
    }
    T _ctrl_dt;
};

template <typename T>
class State {
public:
    State(PrestoeSystem<T>* sys):_sys_info(sys){}
    virtual ~State(){}

    virtual void Initialize() {}
    virtual void OnEnter() = 0;
    virtual void RunNominal() = 0;
    virtual void RunTransition() {}
    virtual Command<T>* GetCommand() = 0;

protected:
    T _state_time = 0.0;
    SystemInfo<T> _sys_info;
};

#endif