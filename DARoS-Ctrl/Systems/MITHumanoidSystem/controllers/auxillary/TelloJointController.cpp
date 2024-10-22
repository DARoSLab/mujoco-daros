#include "TelloJointController.h"

/*!
 * Zero all limb _commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template <typename T>
void TelloJointController<T>::zeroCommand() {
  for (auto& cmd : _commands) {
    cmd->zero();
  }
  _JointsEnabled = false;
}

template <typename T>
void TelloJointController<T>::updateData(const tello_joint_data_lcmt* lcm_data) {

  for (size_t leg(0); leg < tello::num_leg; leg++){
    for(size_t jidx(0); jidx< tello::num_leg_joint; ++jidx){
      _datas[leg]->q(jidx) = lcm_data->q[tello::num_leg_joint *leg  + jidx];
      _datas[leg]->qd(jidx) = lcm_data->qd[tello::num_leg_joint *leg  + jidx];
    }
  }

  size_t idx_offset = tello::num_leg_joint * tello::num_leg;

  for (size_t arm(0); arm < tello::num_arm; arm++){
    for(size_t jidx(0); jidx< tello::num_arm_joint; ++jidx){
      _datas[tello::num_leg + arm]-> q(jidx) = lcm_data-> q[idx_offset + tello::num_arm_joint*arm  + jidx];
      _datas[tello::num_leg + arm]->qd(jidx) = lcm_data->qd[idx_offset + tello::num_arm_joint*arm  + jidx];
    }
  }
}


/*!
 * Update the "limb command" for the (tello) board message
 */
template <typename T>
void TelloJointController<T>::updateCommand(tello_joint_command_lcmt* lcm_cmd) {
  T tauDesMotor, iDes, bemf, vDes, vActual, tauActMotor;

  for (size_t leg = 0; leg < tello::num_leg; leg++) {
    // estimate torque
    DVec<T> legTorqueFF = _commands[leg]->tauFeedForward;
    _datas[leg]->tauEstimate =
      legTorqueFF +
      _commands[leg]->kpJoint * (_commands[leg]->qDes - _datas[leg]->q) +
      _commands[leg]->kdJoint * (_commands[leg]->qdDes - _datas[leg]->qd);
    for (size_t jidx(0); jidx < tello::num_leg_joint; jidx++){
      // tauFF
      lcm_cmd->tau_ff[leg*tello::num_leg_joint + jidx] = _commands[leg]->tauFeedForward(jidx);
      lcm_cmd->q_des[leg*tello::num_leg_joint + jidx] = _commands[leg]->qDes(jidx);
      lcm_cmd->qd_des[leg*tello::num_leg_joint + jidx] = _commands[leg]->qdDes(jidx);

      // actual torque based on actuator limits (this is just to testm so everything is hard coded)
      // tauDesMotor = _datas[leg]->tauEstimate[jidx] / _gr[jidx];                  // motor torque
      // iDes = tauDesMotor / (_kt[jidx] * 1.5);                                   // i = tau / KT
      // bemf = _datas[leg]->qd(jidx) * _gr[jidx] * _kt[jidx] * 2.;                 // back emf
      // vDes = iDes * _R[jidx] + bemf;                                            // v = I*R + emf
      // vActual = coerce(vDes, -_V, _V);                                          // limit to battery voltage
      // tauActMotor = 1.5 * _kt[jidx] * (vActual - bemf) / _R[jidx];              // tau = Kt * I = Kt * V / R
      // _datas[leg]->tauAct[jidx] = _gr[jidx] * coerce(tauActMotor, -_tauMax[jidx], _tauMax[jidx]);
    }
  }

  // No Torso

  // TODO: update arm data 
  for (size_t arm = 0; arm < tello::num_arm; arm++) {
    size_t idx_offset = tello::num_leg_joint * tello::num_leg + tello::num_arm_joint * arm;

    for (size_t jidx = 0; jidx < tello::num_arm_joint; jidx++){
      lcm_cmd->tau_ff[idx_offset + jidx] = _commands[tello::num_leg + arm ]->tauFeedForward(jidx);
      lcm_cmd->q_des[idx_offset + jidx] = _commands[tello::num_leg + arm  ]->qDes(jidx);
      lcm_cmd->qd_des[idx_offset + jidx] = _commands[tello::num_leg + arm ]->qdDes(jidx);
    }
  }
}

/*!
 * Set LCM debug data from limbs _commands and data
 */
template<typename T>
void TelloJointController<T>::setLcm(tello_joint_data_lcmt *lcmData, tello_joint_command_lcmt *lcmCommand) {
  int idx = 0;
  for(size_t clusters = 0; clusters < tello::num_joint_group; clusters++) {
    for(int jidx = 0; jidx < _datas[clusters]->_num_joints; jidx++) {
      lcmData->q[idx] = _datas[clusters]->q[jidx];
      lcmData->qd[idx] = _datas[clusters]->qd[jidx];
      lcmData->tau_est[idx] = _datas[clusters]->tauEstimate[jidx];
      lcmData->tau_act[idx] = _datas[clusters]->tauAct[jidx];
      lcmCommand->tau_ff[idx] = _commands[clusters]->tauFeedForward[jidx];
      lcmCommand->q_des[idx] = _commands[clusters]->qDes[jidx];
      lcmCommand->qd_des[idx] = _commands[clusters]->qdDes[jidx];
      lcmCommand->kp_joint[idx] = _commands[clusters]->kpJoint(jidx, jidx);
      lcmCommand->kd_joint[idx] = _commands[clusters]->kdJoint(jidx, jidx);
      idx++;
    }
  }
}

template class TelloJointController<double>;
template class TelloJointController<float>;

