#ifndef __MiniArm_OBS_MANAGER_H__
#define __MiniArm_OBS_MANAGER_H__

#include <ObserverManager.hpp>
#include <cppTypes.h>
#include <mujoco/mujoco.h>
#include <MiniArmDefinition.h>
#include <pretty_print.h>

enum MiniArmObsList {
  CheaterMode = 0,
  NUM_OBS
};
template<typename T> class CheaterModeObserver;

template<typename T>
class MiniArmObsManager: public ObserverManager<T>{
  public:
    MiniArmObsManager(mjData* data):ObserverManager<T>(){
      this->_observers.resize(MiniArmObsList::NUM_OBS);
      this->_observers[MiniArmObsList::CheaterMode] = new CheaterModeObserver<T>(data);
      printf("[MiniArmObsManager] Constructed\n");
    }
    virtual ~MiniArmObsManager(){
      for(size_t i(0); i<MiniArmObsList::NUM_OBS; i++){
        delete this->_observers[i];
      }
    }
};

template<typename T>
class CheaterModeObserver: public Observer<T>{
  public:
    CheaterModeObserver(mjData* data):
    _q(miniarm::nDoF), _dq(miniarm::nDoF){ 
      _mjData = data; 
      printf("[CheaterModeObserver] Constructed\n");
    }
    virtual ~CheaterModeObserver(){ }

    virtual void Update(){
      // std::cout<<_mjData->qpos<<std::endl;
      for(size_t i(0); i<miniarm::nDoF; i++){
        _q[i] = static_cast<T>(_mjData->qpos[i]);
        _dq[i] = static_cast<T>(_mjData->qvel[i]);
      }
      _q[miniarm::nDoF] = _mjData->qpos[miniarm::nDoF];
    }
    virtual void PrintInfo(){
      pretty_print(_q, std::cout, "CheaterModeObserver: _q");
      pretty_print(_dq, std::cout, "CheaterModeObserver: _dq");
    }
    DVec<T> _q, _dq;

  protected:
    mjData* _mjData;
};


#endif