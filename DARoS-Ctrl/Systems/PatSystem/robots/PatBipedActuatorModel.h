/*! @file ActuatorModel.h
 *  @brief Model of actuator
 *  Includes friction, max torque, and motor torque speed curve.
 *
 *  The getTorque is used for torque at the joint, not torque at the motor.
 *  The provided frictions are for torques at the joint, not torque at the motor
 *  The R/KT are for the motor
 */

#ifndef PAT_ACTUATORMODEL_H
#define PAT_ACTUATORMODEL_H

#include <ActuatorModel.h>
#include <Utilities/utilities.h>

/*!
 * A model of an actuator containing friction and electrical effects
 */
template <typename T>
class PatBipedActuatorModel : public ActuatorModel<T> {
 public:

  /*!
   * Construct a new actuator model with the given parameters
   * @param gearRatio : Gear reduction
   * @param motorKT : Value of KT (torque constant) for the motor
   * @param motorR : Motor resistance
   * @param batteryV : Battery voltage
   * @param damping : Actuator damping (at the joint, Nm/(rad/sec))
   * @param dryFriction : Actuator dry friction (at the joint, Nm)
   * @param tauMax : Maximum torque output of the actuator
   */
  PatBipedActuatorModel(T gearRatio, T motorKT, T motorR, T batteryV,
                        T damping, T dryFriction, T tauMax): ActuatorModel<T>() {
    this->_gr = gearRatio;
    this->_kt = motorKT;
    this->_R = motorR;
    this->_V = batteryV;
    this->_damping = damping;
    this->_dryFriction = dryFriction;
    this->_tauMax = tauMax;
    this->_torque_ff = 0.;
    this->_Kp = 10.;
    this->_Kd = 0.1;
  }

  virtual ~PatBipedActuatorModel() {}

  /*!
   * Compute actual actuator torque, given desired torque and speed.
   * takes into account friction (dry and damping), voltage limits, and torque
   * limits
   * @param tauDes : desired torque
   * @param qd : current actuator velocity (at the joint)
   * @return actual produced torque
   */
  T getTorque (T tauDes, T qd) const {
    // compute motor torque
    T tauDesMotor = tauDes / this->_gr;        // motor torque
    T iDes = tauDesMotor / (this->_kt * 1.5);  // i = tau / KT
    // T bemf =  qd * _gr * _kt * 1.732;     // back emf
    T bemf = qd * this->_gr * this->_kt * 2.;       // back emf
    T vDes = iDes * this->_R + bemf;          // v = I*R + emf
    T vActual = coerce(vDes, -this->_V, this->_V);  // limit to battery voltage
    T tauActMotor =
        1.5 * this->_kt * (vActual - bemf) / this->_R;  // tau = Kt * I = Kt * V / R
    T tauAct = this->_gr * coerce(tauActMotor, -this->_tauMax, this->_tauMax);

    // add damping and dry friction
    if (this->_frictionEnabled)
      tauAct = tauAct - this->_damping * qd - this->_dryFriction * sgn(qd);

    return tauAct;
  }
};

#endif  // PAT_ACTUATORMODEL_H
