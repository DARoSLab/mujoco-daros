/*!
 * @file LiftSwingTrajectory.h
 * @brief Utility to generate cubic joint trajectories.
 *
 */

#ifndef PAT_LIFTSWINGTRAJECTORY_H
#define PAT_LIFTSWINGTRAJECTORY_H

#include "cppTypes.h"

/*!
 * A foot swing trajectory for a single foot
 */
template<typename T>
class LiftSwingTrajectory {
public:

  /*!
   * Construct a new foot swing trajectory with everything set to zero
   */
  LiftSwingTrajectory() {
    _p0.setZero();
    _p_mid.setZero();
    _pf.setZero();
    _p.setZero();
    _pdot.setZero();
    _pddot.setZero();
  }

  /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position
   */
  void setInitialPosition(Vec3<T> p0) {
    _p0 = p0;
  }

  /*!
   * Set the lift location of the foot
   * @param p0 : the lift foot position
   */
  void setMiddlePosition(Vec3<T> p_mid) {
    _p_mid = p_mid;
  }

  /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton
   */
  void setFinalPosition(Vec3<T> pf) {
    _pf = pf;
  }



  void computeLiftSwingTrajectory(T phase, T liftTime, T swingTime);

  /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
  Vec3<T> getPosition() {
    return _p;
  }

  /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
  Vec3<T> getVelocity() {
    return _pdot;
  }

  /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
  Vec3<T> getAcceleration() {
    return _pddot;
  }

private:
  Vec3<T> _p0, _p_mid, _pf, _p, _pdot, _pddot;
};


#endif //PAT_LIFTSWINGTRAJECTORY_H
