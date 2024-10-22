/*! @file ActuatorModel.h
 *  @brief Model of actuator
 *  Includes friction, max torque, and motor torque speed curve.
 *
 *  The getTorque is used for torque at the joint, not torque at the motor.
 *  The provided frictions are for torques at the joint, not torque at the motor
 *  The R/KT are for the motor
 */

#ifndef PROJECT_Tello_ACTUATORMODEL_H
#define PROJECT_Tello_ACTUATORMODEL_H

#include <Dynamics/ActuatorModel.h>
#include <Utilities/utilities.h>

/*!
 * A model of an actuator containing friction and electrical effects
 */
template <typename T>
class TelloActuatorModel : public ActuatorModel<T> {
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
  TelloActuatorModel(T gearRatio, T motorKT, T motorR, T batteryV,
                        T damping, T dryFriction, T tauMax, T flux_linkage,
                        T inductance, T poles): ActuatorModel<T>() {
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
    _flux_linkage = flux_linkage;
    _inductance = inductance;
    _poles = poles;
  }

  virtual ~TelloActuatorModel() {}

  /*!
   * Compute actual actuator torque, given desired torque and speed.
   * takes into account friction (dry and damping), voltage limits, and torque
   * limits
   * @param tauDes : desired torque
   * @param qd : current actuator velocity (at the joint)
   * @return actual produced torque
   */
  T getTorque (T tauDes, T qd) const {

    T tauDesMotor = tauDes / this->_gr; // motor torque
    T speed = qd * this->_gr;     // motor speed
    T tauAct = 0.;

    // NEW
    T mod_idx = 1.15; // change to 1.15 for full field weakening (otherwise 1.0)

    // Make sure speed is not past no load speed
    if (pow(_poles,2)*pow(_flux_linkage,2)*pow(this->_V*mod_idx,2) >= 4*pow(_poles,4)*pow(_flux_linkage,4)*pow(speed,2) ){
      // T numerator = 3*sqrt(pow(_poles,2)*pow(_flux_linkage,2)*pow(this->_V*mod_idx,2) - 4*pow(_poles,4)*pow(_flux_linkage,4)*pow(speed,2));
      // T denominator = 4*sqrt(pow(_inductance,2)*pow(_poles,2)*pow(speed,2)+pow(this->_R,2));
      T tauMotorMax = (3*sqrt(pow(_poles,2)*pow(_flux_linkage,2)*pow(this->_V*mod_idx,2) - 4*pow(_poles,4)*pow(_flux_linkage,4)*pow(speed,2))) 
                        / (4*sqrt(pow(_inductance,2)*pow(_poles,2)*pow(speed,2)+pow(this->_R,2)));
      tauMotorMax = coerce(tauMotorMax, -this->_tauMax, this->_tauMax);


      // //simon added
      // tauMotorMax = this->_tauMax;

      if (tauDes*qd >= 0){ // positive work - limited by torque speed curve
        tauAct = this->_gr * coerce(tauDesMotor, -tauMotorMax, tauMotorMax);
      } else { // negative work - limited by saturation torque
        tauAct = this->_gr * coerce(tauDesMotor, -this->_tauMax, this->_tauMax);
      }
    }
    
    // add damping and dry friction
    if (this->_frictionEnabled)
      tauAct = tauAct - this->_damping * qd - this->_dryFriction * sgn(qd);


    // Simon manul added for debugging
    // tauAct = tauDes;
    return tauAct;
  }

 protected:
  T _flux_linkage, _inductance, _poles;

};

#endif  // PROJECT_Tello_ACTUATORMODEL_H
