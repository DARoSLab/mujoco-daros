#ifndef __PRESTOE_SYSTEM_H__
#define __PRESTOE_SYSTEM_H__

#include <System.hpp>

template<typename T>
class PrestoeSystem: public System<T>{
  public:
    PrestoeSystem(const std::string & setup_file);
    virtual ~PrestoeSystem(){ }

    bool Initialization();
    virtual void runCtrl();

    // ObserverManager<T> _obs_manager;
    // UserInputManager<T> _user_input_manager;
    // VisualManager<T> _vis_manager;

  protected:
    bool _initialized = false;
};

#endif