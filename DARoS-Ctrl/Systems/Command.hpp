#ifndef __COMMAND_H__
#define __COMMAND_H__

#include <cppTypes.h>

enum CommandType{
  JTORQUE_CMD = 0,
  JPOS_CMD = 1,
  JTORQUE_POS_CMD = 2,
  NUM_CMD_TYPES
};

template <typename T>
class Command {
  public:
    Command() {}
    virtual ~Command() {}

    CommandType _type;
};

template <typename T>
class JTorqueCommand: public Command<T> {
  public:
    JTorqueCommand(size_t dim): _jtorque_cmd(dim), _dim(dim) {
      this->_type = JTORQUE_CMD;
      _jtorque_cmd.setZero();
    }
    virtual ~JTorqueCommand() {}

    void SetJointCommand(const DVec<T> & jtorque) { _jtorque_cmd = jtorque; }
    void GetJointCommand(DVec<T> & jtorque_output){ jtorque_output = _jtorque_cmd; } 

    DVec<T> _jtorque_cmd;
    size_t _dim;
};

template <typename T>
class JPosCommand : public Command<T> {
  public:
    JPosCommand(size_t dim): _jpos_cmd(dim), _jvel_cmd(dim),
     _Kp(dim), _Kd(dim), _dim(dim) {
      this->_type = JPOS_CMD;

      _jpos_cmd.setZero();
      _jvel_cmd.setZero();
      _Kp.setZero();
      _Kd.setZero();
    }
    virtual ~JPosCommand() {}

    void SetJointCommand(const DVec<T> & jpos_cmd, const DVec<T> & jvel_cmd, 
                           const DVec<T> & Kp, const DVec<T> & Kd) {
      _jpos_cmd = jpos_cmd;
      _jvel_cmd = jvel_cmd;
      _Kp = Kp;
      _Kd = Kd;
    }

    void ComputeTorqueCommand(DVec<T> & jtorque_output, const DVec<T> & jpos, const DVec<T> & jvel) {
      jtorque_output = _Kp.cwiseProduct(_jpos_cmd - jpos) + _Kd.cwiseProduct(_jvel_cmd - jvel);
    }

    DVec<T> _jpos_cmd, _jvel_cmd;
    DVec<T> _Kp, _Kd;
    size_t _dim;
};

template <typename T>
class JTorquePosCommand : public Command<T> {
  public:
    JTorquePosCommand(size_t dim): 
    _jtorque_cmd(dim), _jpos_cmd(dim), _jvel_cmd(dim),
    _Kp(dim), _Kd(dim), _dim(dim) {
      this->_type = JTORQUE_POS_CMD;
      _jtorque_cmd.setZero();
      _jpos_cmd.setZero();
      _jvel_cmd.setZero();
      _Kp.setZero();
      _Kd.setZero();
    }
    virtual ~JTorquePosCommand() {}

    void SetJointCommand(const DVec<T> & jtorque_cmd, const DVec<T> & jpos_cmd, const DVec<T> & jvel_cmd, 
                         const DVec<T> & Kp, const DVec<T> & Kd) {
      _jtorque_cmd = jtorque_cmd;
      _jpos_cmd = jpos_cmd;
      _jvel_cmd = jvel_cmd;
      _Kp = Kp;
      _Kd = Kd;
    }
    void ComputeTorqueCommand(DVec<T> & jtorque_output, const DVec<T> & jpos, const DVec<T> & jvel) {
      jtorque_output = _jtorque_cmd + _Kp.cwiseProduct(_jpos_cmd - jpos) + _Kd.cwiseProduct(_jvel_cmd - jvel);
    }

    DVec<T> _jtorque_cmd, _jpos_cmd, _jvel_cmd;
    DVec<T> _Kp, _Kd;
    size_t _dim;
};

#endif // __COMMAND_H__