#ifndef __MINIARM_CTRL_STATE_H__
#define __MINIARM_CTRL_STATE_H__

#include <cppTypes.h>
#include <Configuration.h>
#include <MiniArmDefinition.h>
#include <MiniArmSystem.hpp>

template<typename T> class Command;

template <typename T> 
class SystemInfo{
public:
    SystemInfo(MiniArmSystem<T>* system){
        _ctrl_dt = system->getCtrlDt();
    }
    T _ctrl_dt;
};

template <typename T>
class State {
public:
    State(MiniArmSystem<T>* sys):_sys_info(sys){}
    virtual ~State(){}

    virtual void OnEnter() = 0;
    virtual void RunNominal() = 0;
    virtual Command<T>* GetCommand() = 0;

protected:
    T _state_time = 0.0;
    SystemInfo<T> _sys_info;
};

#endif