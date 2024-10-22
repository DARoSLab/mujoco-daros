#ifndef __PRESTOE_OBS_MANAGER_H__
#define __PRESTOE_OBS_MANAGER_H__

#include <ObserverManager.hpp>
#include <cppTypes.h>
#include <mujoco/mujoco.h>
#include <PrestoeDefinition.h>
#include <pretty_print.h>

enum PrestoeObsList {
  CheaterMode = 0,
  NUM_OBS
};
template<typename T> class CheaterModeObserver;

template<typename T>
class PrestoeObsManager: public ObserverManager<T>{
  public:
    PrestoeObsManager(mjData* data):ObserverManager<T>(){
      this->_observers.resize(PrestoeObsList::NUM_OBS);
      this->_observers[PrestoeObsList::CheaterMode] = new CheaterModeObserver<T>(data);
      printf("[PrestoeObsManager] Constructed\n");
    }
    virtual ~PrestoeObsManager(){
      for(size_t i(0); i<PrestoeObsList::NUM_OBS; i++){
        delete this->_observers[i];
      }
    }
};

template<typename T>
class CheaterModeObserver: public Observer<T>{
  public:
    CheaterModeObserver(mjData* data):
    _q(prestoe::nDOF+1), _dq(prestoe::nDOF){ 
      _mjData = data; 
      printf("[CheaterModeObserver] Constructed\n");
    }
    virtual ~CheaterModeObserver(){ }

    virtual void Update(){
      for(size_t i(0); i<prestoe::nDOF; i++){
        _q[i] = static_cast<T>(_mjData->qpos[i]);
        _dq[i] = static_cast<T>(_mjData->qvel[i]);
      }
      _q[prestoe::nDOF] = _mjData->qpos[prestoe::nDOF];
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