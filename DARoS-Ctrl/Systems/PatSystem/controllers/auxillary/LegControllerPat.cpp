/*! @file LegControllerPat.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#include "LegControllerPat.h"

/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void LegControllerPatCommand<T>::zero() {
  tauFeedForward.setZero();
  forceFeedForward.setZero();
  qDes.setZero();
  qdDes.setZero();
  pDes.setZero();
  vDes.setZero();
  kpCartesian.setZero();
  kdCartesian.setZero();
  kpJoint.setZero();
  kdJoint.setZero();
}

/*!
 * Zero the leg data
 */
template <typename T>
void LegControllerPatData<T>::zero() {
  this->q.setZero();
  this->qd.setZero();
  this->p.setZero();
  this->v.setZero();
  this->J.setZero();
  this->tauEstimate.setZero();
  this->tauAct.setZero();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template <typename T>
void LegControllerPat<T>::zeroCommand() {
  for (auto& cmd : commands) {
    cmd.zero();
  }
  _legsEnabled = false;
}

/*!
 * Set the leg to edamp.  This overwrites all command data and generates an
 * emergency damp command using the given gain. For the mini-cheetah, the edamp
 * gain is Nm/(rad/s), and for the Cheetah 3 it is N/m. You still must call
 * updateCommand for this command to end up in the low-level command data!
 */
template <typename T>
void LegControllerPat<T>::edampCommand(RobotType robot, T gain) {
  zeroCommand();
  if (robot == RobotType::PATROCLUS) {
    for (size_t leg = 0; leg < pat_biped::num_legs; leg++) {
      for (size_t axis = 0; axis < 3; axis++) {
        commands[leg].kdCartesian(axis, axis) = gain;
      }
    }
  } else {
    assert(false);
  }
}

/*!
 * Update the "leg data" from a SPIne board message
 */
template <typename T>
void LegControllerPat<T>::updateData(const can_data_lcmt* canData) {
  Vec3<T> fb_pos;
  Vec3<T> fb_vel;

  DMat<T> fb_J; fb_J.resize(3,3); fb_J.setZero();

  _FK->UpdateModel(canData);
  for (size_t leg = 0; leg < pat_biped::num_legs; leg++) {
    // q:
    datas[leg]->q(0) = canData->q_abad[leg];
    datas[leg]->q(1) = canData->q_hip[leg];
    datas[leg]->q(2) = canData->q_knee[leg];

    // qd
    datas[leg]->qd(0) = canData->qd_abad[leg];
    datas[leg]->qd(1) = canData->qd_hip[leg];
    datas[leg]->qd(2) = canData->qd_knee[leg];

    _FK->computeFootPosVelJacobian(leg, &datas[leg]->p, &datas[leg]->v, &datas[leg]->J);
    // datas[leg]->p -= _pat.getHipLocation(leg);
    // datas[leg]->v = datas[leg]->J * datas[leg]->qd;
  }
}
/*!
 * Update orientation and angular velocity of the Pat_Kinematics model
 */

template <typename T>
void LegControllerPat<T>::updateModelOriAndVel(Vec4<T> q, Vec3<T> omegaBody){
  _FK->_state.bodyOrientation = q;
  _FK->_state.bodyVelocity.head(3) = omegaBody;
}
/*!
 * Update the "leg command" for the SPIne board message
 */
template <typename T>
void LegControllerPat<T>::updateCommand(can_command_lcmt* canCommand) {

  for (size_t leg = 0; leg < pat_biped::num_legs; leg++) {
    // tauFF
    Vec3<T> legTorque = commands[leg].tauFeedForward;

    // forceFF
    Vec3<T> footForce = commands[leg].forceFeedForward;

    // cartesian PD
    footForce +=
        commands[leg].kpCartesian * (commands[leg].pDes - datas[leg]->p);
    footForce +=
        commands[leg].kdCartesian * (commands[leg].vDes - datas[leg]->v);

    // Torque
    legTorque += datas[leg]->J.transpose() * footForce;

    // set command:
    canCommand->tau_abad_ff[leg] = legTorque(0);
    canCommand->tau_hip_ff[leg] = legTorque(1);
    canCommand->tau_knee_ff[leg] = legTorque(2);

    // joint space pd
    // joint space PD
    canCommand->kd_abad[leg] = commands[leg].kdJoint(0, 0);
    canCommand->kd_hip[leg] = commands[leg].kdJoint(1, 1);
    canCommand->kd_knee[leg] = commands[leg].kdJoint(2, 2);

    canCommand->kp_abad[leg] = commands[leg].kpJoint(0, 0);
    canCommand->kp_hip[leg] = commands[leg].kpJoint(1, 1);
    canCommand->kp_knee[leg] = commands[leg].kpJoint(2, 2);

    canCommand->q_des_abad[leg] = commands[leg].qDes(0);
    canCommand->q_des_hip[leg] = commands[leg].qDes(1);
    canCommand->q_des_knee[leg] = commands[leg].qDes(2);

    canCommand->qd_des_abad[leg] = commands[leg].qdDes(0);
    canCommand->qd_des_hip[leg] = commands[leg].qdDes(1);
    canCommand->qd_des_knee[leg] = commands[leg].qdDes(2);

    // estimate torque
    datas[leg]->tauEstimate =
        legTorque +
        commands[leg].kpJoint * (commands[leg].qDes - datas[leg]->q) +
        commands[leg].kdJoint * (commands[leg].qdDes - datas[leg]->qd);

    // actual torque based on actuator limits
    T tauDesMotor, iDes, bemf, vDes, vActual, tauActMotor;
    for (size_t jidx = 0; jidx < 3; jidx++){
      tauDesMotor = datas[leg]->tauEstimate[jidx] / _gr[jidx];                  // motor torque
      iDes = tauDesMotor / (_kt[jidx] * 1.5);                                   // i = tau / KT
      bemf = datas[leg]->qd(jidx) * _gr[jidx] * _kt[jidx] * 2.;                 // back emf
      vDes = iDes * _R[jidx] + bemf;                                            // v = I*R + emf
      vActual = coerce(vDes, -_V, _V);                                          // limit to battery voltage
      tauActMotor = 1.5 * _kt[jidx] * (vActual - bemf) / _R[jidx];              // tau = Kt * I = Kt * V / R
      datas[leg]->tauAct[jidx] = _gr[jidx] * coerce(tauActMotor, -_tauMax[jidx], _tauMax[jidx]);
    }

    // canCommand->flags[leg] = _legsEnabled ? 1 : 0;

    //std::cout<<leg<<": "<<datas[leg]->tauEstimate<<std::endl;
    //std::cout<<leg<<": "<<legTorque<<std::endl;
    //std::cout<<"force ff: "<<commands[leg].forceFeedForward<<std::endl;
  }
}

/*!
 * Set LCM debug data from leg commands and data
 */
template<typename T>
void LegControllerPat<T>::setLcm(
    pat_leg_control_data_lcmt *lcmData,
    pat_leg_control_command_lcmt *lcmCommand) {

    for(size_t leg = 0; leg < pat_biped::num_legs; leg++) {
        for(size_t axis = 0; axis < 3; axis++) {
            size_t idx = leg*3 + axis;
            lcmData->q[idx] = datas[leg]->q[axis];
            lcmData->qd[idx] = datas[leg]->qd[axis];
            lcmData->p[idx] = datas[leg]->p[axis];
            lcmData->v[idx] = datas[leg]->v[axis];
            lcmData->tau_est[idx] = datas[leg]->tauEstimate[axis];
            lcmData->tau_act[idx] = datas[leg]->tauAct[axis];

            lcmCommand->tau_ff[idx] = commands[leg].tauFeedForward[axis];
            lcmCommand->f_ff[idx] = commands[leg].forceFeedForward[axis];
            lcmCommand->q_des[idx] = commands[leg].qDes[axis];
            lcmCommand->qd_des[idx] = commands[leg].qdDes[axis];
            lcmCommand->p_des[idx] = commands[leg].pDes[axis];
            lcmCommand->v_des[idx] = commands[leg].vDes[axis];
            lcmCommand->kp_cartesian[idx] = commands[leg].kpCartesian(axis, axis);
            lcmCommand->kd_cartesian[idx] = commands[leg].kdCartesian(axis, axis);
            lcmCommand->kp_joint[idx] = commands[leg].kpJoint(axis, axis);
            lcmCommand->kd_joint[idx] = commands[leg].kdJoint(axis, axis);
        }
    }
}

template struct LegControllerPatCommand<double>;
template struct LegControllerPatCommand<float>;

template struct LegControllerPatData<double>;
template struct LegControllerPatData<float>;

template class LegControllerPat<double>;
template class LegControllerPat<float>;

/*!
 * Compute the position of the foot and its Jacobian.  This is done in the local
 * leg coordinate system. If J/p are NULL, the calculation will be skipped.
 */
template <typename T>
void computeLegJacobianAndPosition(PatBiped<T>& biped, DVec<T>& q, DMat<T>* J,
                                   Vec3<T>* p, int leg) {


  T l1 = biped._abadLinkLength;
  T l2 = biped._hipLinkLength;
  T l3 = biped._kneeLinkLength;
  T l4 = biped._kneeLinkY_offset;
  T sideSign = biped.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  if (J) {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p) {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

template void computeLegJacobianAndPosition<double>(PatBiped<double>& biped,
                                                    DVec<double>& q,
                                                    DMat<double>* J,
                                                    Vec3<double>* p, int leg);
template void computeLegJacobianAndPosition<float>(PatBiped<float>& biped,
                                                   DVec<float>& q,
                                                   DMat<float>* J,
                                                   Vec3<float>* p, int leg);
