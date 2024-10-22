/*!
 * @file GaitScheduler.h
 * @brief Logic for fixed-gait timing
 */


// SEHWAN TODO:
// make event-based gait switching FSM in this class
// test with MPC?
// how to deal with phases during unexpected contact? set to 0.0/1.0 on transitions from expected?


#ifndef GUIDE_DOG_GAIT_SCHEDULER_H
#define GUIDE_DOG_GAIT_SCHEDULER_H

#include <iostream>

#include "cppTypes.h"
#include <dynamics/Robot.h>
#include <PatParameters.h>

/**
 * Enumerated gait types. Preplanned gaits are defined.
 */
class GaitTypePat {
public:
  static constexpr size_t STAND = 0;
  static constexpr size_t STAND_CYCLE = 1;
  static constexpr size_t STATIC_WALK = 2;
  static constexpr size_t AMBLE = 3;
  static constexpr size_t TROT_WALK = 4;
  static constexpr size_t TROT = 5;
  static constexpr size_t TROT_RUN = 6;
  static constexpr size_t PACE = 7;
  static constexpr size_t BOUND = 8;
  static constexpr size_t ROTARY_GALLOP = 9;
  static constexpr size_t TRAVERSE_GALLOP = 10;
  static constexpr size_t PRONK = 11;
  static constexpr size_t THREE_FOOT = 12;
  static constexpr size_t TWO_FOOT=13;
  static constexpr size_t CUSTOM = 14;
  static constexpr size_t TRANSITION_TO_STAND = 15;
  static constexpr size_t WALK = 16;
  static constexpr size_t RUN = 17;
  static constexpr size_t HOP = 18;
};

/**
 * Timing data for a gait
 */
template <typename T>
struct GaitDataPat {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GaitDataPat() { }

  // Zero out all of the data
  void zero();

  // The current GaitTypePat
  size_t _currentGait;

  // Next GaitTypePat to transition into
  size_t _nextGait;

  // Gait name string
  std::string gaitName;

  // Number of feet available for balance
  int NUM_FEET;
  int NUM_CONTROLLED_LIMBS;
  int NUM_CONTACT_POINTS;
  DiVec limb_contact_points;

  // Gait descriptors
  T periodTimeNominal;      // overall period time to scale
  T initialPhase;           // initial phase to offset
  T switchingPhaseNominal;  // nominal phase to switch contacts
  int overrideable;

  // Track the number of scheduled stance feet
  int num_stance_feet_scheduled = 0;

  // Enable flag for each foot
  DiVec gaitEnabled;  // enable gait controlled legs

  // Time based descriptors
  DVec<T> periodTime;           // overall foot scaled gait period time
  DVec<T> timeStance;           // total stance time
  DVec<T> timeSwing;            // total swing time
  DVec<T> timeStanceRemaining;  // stance time remaining
  DVec<T> timeSwingRemaining;   // swing time remaining

  // Phase based descriptors
  DVec<T> switchingPhase;  // phase to switch to swing
  DVec<T> phaseVariable;   // overall gait phase for each foot
  DVec<T> phaseOffset;     // nominal gait phase offsets
  DVec<T> phaseScale;      // phase scale relative to variable
  DVec<T> phaseStance;     // stance subphase
  DVec<T> phaseSwing;      // swing subphase

  // Scheduled contact states
  DiVec contactStateScheduled;  // contact state of the foot
  DiVec contactStatePrev;       // previous contact state of the foot
  DiVec touchdownScheduled;     // scheduled touchdown event flag
  DiVec liftoffScheduled;       // scheduled liftoff event flag
};

/**
 * Utility to process GaitData and schedule foot steps and swings.
 */
template <typename T>
class GaitSchedulerPat {
 public:
  // Constructors for the GaitScheduler
  GaitSchedulerPat(PatParameters* _userParameters, float _dt, Robot<T>* _robot);
  ~GaitSchedulerPat(){};

  // Initialize the Gait Scheduler
  void initialize();

  // Iteration step for scheduler logic
  void step();

  // Creates a new gait from predefined library
  void multipleContacts();
  void modifyGait();
  void createGait();
  void calcAuxiliaryGaitData();
  void initializePatGait();
  void initializeBipedGait();
  void initializeBipedGait2();

  // Prints the characteristic info and curret state
  void printGaitInfo();

  // Struct containing all of the gait relevant data
  GaitDataPat<T> gaitData;

  // Natural gait modifiers
  T period_time_natural = 0.5;
  T switching_phase_natural = 0.5;
  T swing_time_natural = 0.25;

 private:
  // The quadruped model
  // Pat<T>& _quadruped;
  PatParameters* userParameters;

  // Number of feet possible for the gaits

  Robot<T>* robot;

  // Control loop timestep change
  T dt;

  // Phase change at each step
  T dphase;

  // Choose how often to print info, every N iterations
  int printNum = 5;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;
};



#endif
