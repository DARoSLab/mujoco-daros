#ifndef __OBSERVER_MANAGER_H__
#define __OBSERVER_MANAGER_H__

#include <iostream>

template<typename T>
class Observer{
  public:
    Observer(){ }
    virtual ~Observer(){ }
    virtual void Update() = 0;
    virtual void PrintInfo(){} 
};

template<typename T>
class ObserverManager{
  public:
    ObserverManager(){ 
      this->_observers.clear();
    }
    virtual ~ObserverManager(){ }

    void UpdateObservers(){
      for (size_t i(0); i < this->_observers.size(); i++){
        this->_observers[i]->Update();
      }
    }
    std::vector<Observer<T> *> _observers;
};

#endif