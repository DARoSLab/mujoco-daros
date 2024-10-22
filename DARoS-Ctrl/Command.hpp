#ifndef __COMMAND_H__
#define __COMMAND_H__

#include <cppTypes.h>

enum CommandType{
  jpos_command = 0,
  jtorque_command = 1,
  jtorque_pos_command = 2,
  num_command
};

template <typename T>
class Command {
  public:
    Command() {}
    virtual ~Command() {}

    CommandType _type;
};

template <typename T>
class JPosCommand : public Command<T> {
  public:
    JPosCommand(size_t dim): _jpos_cmd(dim), _dim(dim) {
      this->_type = jpos_command;
    }
    virtual ~JPosCommand() {}

    DVec<T> _jpos_cmd;
    size_t _dim;
};

template <typename T>
class JTorqueCommand : public Command<T> {
  public:
    JTorqueCommand(size_t dim): _jtorque_cmd(dim), _dim(dim) {
      this->_type = jtorque_command;
    }
    virtual ~JTorqueCommand() {}

    void SetJointPDCommand(const DVec<T> & jpos_cmd, const DVec<T> & jvel_cmd, 
                           const DVec<T> & jpos, const DVec<T> & jvel,
                           const DVec<T> & Kp, const DVec<T> & Kd) {
      _jtorque_cmd = Kp.cwiseProduct(jpos_cmd - jpos) + Kd.cwiseProduct(jvel_cmd - jvel);
    }

    DVec<T> _jtorque_cmd;
    size_t _dim;
};

template <typename T>
class JTorquePosCommand : public Command<T> {
  public:
    JTorquePosCommand(size_t dim): 
    _jtorque_cmd(dim), _jpos_cmd(dim), _jvel_cmd(dim),
    _Kp(dim), _Kd(dim), _dim(dim) {
      this->_type = jtorque_pos_command;
    }
    virtual ~JTorquePosCommand() {}

    void SetJointCommand(const DVec<T> & jtorque_cmd, const DVec<T> & jpos_cmd, const DVec<T> & jvel_cmd, 
                         const DVec<T> & jpos, const DVec<T> & jvel,
                         const DVec<T> & Kp, const DVec<T> & Kd) {
      _jtorque_cmd = jtorque_cmd;
      _jpos_cmd = jpos_cmd;
      _jvel_cmd = jvel_cmd;
      _Kp = Kp;
      _Kd = Kd;
    }

    DVec<T> _jtorque_cmd, _jpos_cmd, _jvel_cmd;
    DVec<T> _Kp, _Kd;
    size_t _dim;
};

#endif // __COMMAND_H__