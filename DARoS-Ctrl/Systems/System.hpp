#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <string>

template <typename T>
class System {
  public:
    System (const std::string & setup_file) {};
    virtual ~System() {}

    virtual void runCtrl() = 0;

    T getCtrlDt() { return _ctrl_dt; }
    std::string getMujocoFile() { return _mujoco_file; }

  protected:
    T _ctrl_dt = 0.001;
    std::string _mujoco_file;
};



#endif // __SYSTEM_H__