/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */
#ifdef MOCAP_BUILD
#ifndef PROJECT_MOCAPESTIMATOR_H
#define PROJECT_MOCAPESTIMATOR_H

#include "StateEstimatorContainer.h"
#include <robots/PatBiped.h>
#include <iostream>
#include <cstring>
#include <thread>
#include "../../third-parties/phase_space/owl.hpp"
#define Z_OFFSET 0.127
#define TIMEOUT 1000 //1MS
/*!
 * Position and velocity estimator based on MoCap data.
 */
template <typename T>
class MoCapEstimator : public GenericEstimator<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MoCapEstimator();
  virtual void run();
  virtual void setup();
 private:
   OWL::Context owl;
   OWL::Markers markers;
   OWL::Rigids rigids;
   std::string address = "192.168.1.230";
   Vec3<T> _body_position_est, _prev_body_position_est;
   Vec3<T> _body_velocity_est;
   Vec4<T> _body_orientation_est;
   bool first_visit = true;
   std::thread _mc_thread;
   void _pat_mocap_thread();
};

/*!
 * "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 */
template<typename T>
class CheaterMoCapEstimator : public GenericEstimator<T> {
public:
  virtual void run();
  virtual void setup() {}
};

#endif  // PROJECT_POSITIONVELOCITYESTIMATOR_H
#endif // MOCAP_BUILD
