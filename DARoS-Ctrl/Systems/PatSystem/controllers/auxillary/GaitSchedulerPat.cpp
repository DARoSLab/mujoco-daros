/*!
 * @file GaitScheduler.cpp
 * @brief Logic for fixed-gait timing
 */

/*========================= Gait Scheduler ============================*/
/**
 *
 */

#include "GaitSchedulerPat.h"

/*=========================== Gait Data ===============================*/
/**
 * Reset gait data to zeros
 */
template <typename T>
void GaitDataPat<T>::zero() {
  // Stop any gait transitions
  _nextGait = _currentGait;

  // General Gait descriptors
  periodTimeNominal = 0.0;      // overall period time to scale
  initialPhase = 0.0;           // initial phase to offset
  switchingPhaseNominal = 0.0;  // nominal phase to switch contacts
  overrideable = 0;             // allows the gait parameters to be overridden

  // Track the number of scheduled stance feet
  num_stance_feet_scheduled = 0;

  // Enable flag for each foot
  gaitEnabled = DiVec::Zero(NUM_FEET);  // enable gait controlled legs

  // Time based descriptors
  periodTime = DVec<T>::Zero(NUM_FEET);           // overall gait period time
  timeStance = DVec<T>::Zero(NUM_FEET);           // total stance time
  timeSwing = DVec<T>::Zero(NUM_FEET);            // total swing time
  timeStanceRemaining = DVec<T>::Zero(NUM_FEET);  // stance time remaining
  timeSwingRemaining = DVec<T>::Zero(NUM_FEET);   // swing time remaining

  // Phase based descriptors
  switchingPhase = DVec<T>::Zero(NUM_FEET);  // phase to switch to swing
  phaseVariable = DVec<T>::Zero(NUM_FEET);   // overall gait phase for each foot
  phaseOffset = DVec<T>::Zero(NUM_FEET);     // nominal gait phase offsets
  phaseScale = DVec<T>::Zero(NUM_FEET);      // phase scale relative to variable
  phaseStance = DVec<T>::Zero(NUM_FEET);     // stance subphase
  phaseSwing = DVec<T>::Zero(NUM_FEET);      // swing subphase

  // Scheduled contact states
  contactStateScheduled = DiVec::Zero(NUM_FEET);  // contact state of the foot
  contactStatePrev = DiVec::Zero(NUM_FEET);       // previous contact state of the foot
  touchdownScheduled = DiVec::Zero(NUM_FEET);     // scheduled touchdown flag
  liftoffScheduled = DiVec::Zero(NUM_FEET);       // scheduled liftoff flag
}

template struct GaitDataPat<double>;
template struct GaitDataPat<float>;

/*========================= Gait Scheduler ============================*/

/**
 * Constructor to automatically setup a basic gait
 */
template <typename T>
GaitSchedulerPat<T>::GaitSchedulerPat(PatParameters* _userParameters, float _dt, Robot<T>* _robot) {
  printf("[GaitScheduler Pat] In\n");
  userParameters = _userParameters;
  dt = _dt;
  robot = _robot;
  gaitData.NUM_FEET = robot->NUM_FEET;
  gaitData.NUM_CONTROLLED_LIMBS = robot->NUM_CONTROLLED_LIMBS;
  gaitData.NUM_CONTACT_POINTS = robot->NUM_CONTACT_POINTS;
  gaitData.limb_contact_points = robot->limb_contact_points;
  gaitData.zero();
  printf("[GaitScheduler Pat] Initilizing\n");
  initialize();
  printf("[GaitScheduler Pat] Constructed\n");
}

/**
 * Initialize the gait data
 */
template <typename T>
void GaitSchedulerPat<T>::initialize() {
  std::cout << "[GAIT] Initialize Gait Scheduler" << std::endl;

  // Start the gait in a trot since we use this the most
  gaitData._currentGait = GaitTypePat::STAND;

  // Zero all gait data
  gaitData.zero();

  createGait();

  period_time_natural = gaitData.periodTimeNominal;
  switching_phase_natural = gaitData.switchingPhaseNominal;
}

/**
 * Executes the Gait Schedule step to calculate values for the defining
 * gait parameters.
 */
template <typename T>
void GaitSchedulerPat<T>::step() {
  // Modify the gait with settings
  modifyGait();

  if (gaitData._currentGait != GaitTypePat::STAND) {
    // Track the reference phase variable
    gaitData.initialPhase = fmod((gaitData.initialPhase +
          (dt / gaitData.periodTimeNominal)), 1);
  }

  // Start the counter of stance feet at 0
  gaitData.num_stance_feet_scheduled = 0;

  // Iterate over the feet
  for (int foot = 0; foot < gaitData.NUM_FEET; foot++) {
    // Set the previous contact state for the next timestep
    gaitData.contactStatePrev(foot) = gaitData.contactStateScheduled(foot);

    if (gaitData.gaitEnabled(foot) == 1) {
      // Monotonic time based phase incrementation
      if (gaitData._currentGait == GaitTypePat::STAND) {
        // Don't increment the phase when in stand mode
        dphase = 0.0;
      } else {
        dphase = gaitData.phaseScale(foot) * (dt / gaitData.periodTimeNominal);
      }

      // Find each foot's current phase
      gaitData.phaseVariable(foot) =
        fmod((gaitData.phaseVariable(foot) + dphase), 1);

      // Check the current contact state
      if (gaitData.phaseVariable(foot) <= gaitData.switchingPhase(foot)) {
        // Foot is scheduled to be in contact
        gaitData.contactStateScheduled(foot) = 1;

        // Count the number of stance feet
        gaitData.num_stance_feet_scheduled++;

        // Stance subphase calculation
        gaitData.phaseStance(foot) =
          gaitData.phaseVariable(foot) / gaitData.switchingPhase(foot);

        // Swing phase has not started since foot is in stance
        gaitData.phaseSwing(foot) = 0.0;

        // Calculate the remaining time in stance
        gaitData.timeStanceRemaining(foot) =
          gaitData.periodTime(foot) *
          (gaitData.switchingPhase(foot) - gaitData.phaseVariable(foot));

        // Foot is in stance, no swing time remaining
        gaitData.timeSwingRemaining(foot) = 0.0;

        // First contact signifies scheduled touchdown
        if (gaitData.contactStatePrev(foot) == 0) {
          // Set the touchdown flag to 1
          gaitData.touchdownScheduled(foot) = 1;

        } else {
          // Set the touchdown flag to 0
          gaitData.touchdownScheduled(foot) = 0;
        }

      } else {
        // Foot is not scheduled to be in contact
        gaitData.contactStateScheduled(foot) = 0;

        // Stance phase has completed since foot is in swing
        gaitData.phaseStance(foot) = 1.0;

        // Swing subphase calculation
        gaitData.phaseSwing(foot) =
          (gaitData.phaseVariable(foot) - gaitData.switchingPhase(foot)) /
          (1.0 - gaitData.switchingPhase(foot));

        // Foot is in swing, no stance time remaining
        gaitData.timeStanceRemaining(foot) = 0.0;

        // Calculate the remaining time in swing
        gaitData.timeSwingRemaining(foot) =
          gaitData.periodTime(foot) * (1 - gaitData.phaseVariable(foot));

        // First contact signifies scheduled touchdown
        if (gaitData.contactStatePrev(foot) == 1) {
          // Set the liftoff flag to 1
          gaitData.liftoffScheduled(foot) = 1;

        } else {
          // Set the liftoff flag to 0
          gaitData.liftoffScheduled(foot) = 0;
        }
      }

    } else {
      // Leg is not enabled
      gaitData.phaseVariable(foot) = gaitData.switchingPhase(foot);

      // Foot is not scheduled to be in contact
      gaitData.contactStateScheduled(foot) = 0;
    }
  }
}



/**
 *
 */
template <typename T>
void GaitSchedulerPat<T>::multipleContacts() {
  /*
  int i_cp = 0;
  foot_contacts = DiVec::Zero(NUM_FEET);
  foot_contacts << 2, 2;
  for (int foot = 0; foot < NUM_FEET; foot++) {
    T Phi_total_foot = 0;
    T Phi_offset_diff_max = 0;
    T Phi_offset_last = ;
    for (int cp = 0; cp < foot_contacts(foot); cp++) {
      if (cp == 0) {

      }
    }
  }
  */
  T Ts_foot;
  int i_cp = 0;
  for (int foot = 0; foot < gaitData.NUM_FEET; foot++) {
    if (abs((gaitData.phaseOffset(i_cp + 1)) + gaitData.phaseOffset(i_cp))) {
      Ts_foot = gaitData.periodTime(foot) *
                (abs((gaitData.phaseOffset(i_cp + 1)) +
                     gaitData.phaseOffset(i_cp)) +
                 gaitData.switchingPhase(foot));

    } else {
      if (abs((gaitData.phaseOffset(i_cp + 1)) < gaitData.phaseOffset(i_cp))) {
        Ts_foot = gaitData.periodTime(foot) *
                  (abs((gaitData.phaseOffset(i_cp)) +
                       (1 + gaitData.phaseOffset(i_cp + 1))) +
                   gaitData.switchingPhase(foot));

      } else {
        Ts_foot = gaitData.periodTime(foot) *
                  (abs((1 + gaitData.phaseOffset(i_cp)) +
                       (gaitData.phaseOffset(i_cp + 1))) +
                   gaitData.switchingPhase(foot));
      }
    }
    gaitData.timeStance(foot) = Ts_foot;
    i_cp = i_cp + gaitData.limb_contact_points(foot);
  }
}



/**
 *
 */
template <typename T>
void GaitSchedulerPat<T>::modifyGait() {
  // Choose the gaits and parameters as selected
  switch ((int)userParameters->gait_override) {
  case 0:
    // Use default gait and default settings from Control code
    if (gaitData._currentGait != gaitData._nextGait) {
      createGait();
    }
    break;

  case 1:
    // Use chosen gait with default settings
    if (gaitData._currentGait != (size_t)(userParameters->gait_type)) {
      gaitData._nextGait = (size_t)(userParameters->gait_type);
      createGait();
    }
    break;

  case 2:
    // Use chosen gait and chosen settings
    // Change the gait
    if (gaitData._currentGait != (size_t)(userParameters->gait_type)) {
      gaitData._nextGait = (size_t)(userParameters->gait_type);
      createGait();
    }

    // Adjust the gait parameters
    if (fabs(gaitData.periodTimeNominal - (T)userParameters->gait_period_time) > 0.0001 ||
        fabs(gaitData.switchingPhaseNominal - (T)userParameters->gait_switching_phase) > 0.0001) {
      // Modify the gait with new parameters if it is overrideable
      if (gaitData.overrideable == 1) {
        gaitData.periodTimeNominal = userParameters->gait_period_time;
        gaitData.switchingPhaseNominal = userParameters->gait_switching_phase;
        calcAuxiliaryGaitData();
      }
    }
    break;

  case 3:
    // Use NaturalGaitModification from FSM_State
    if (gaitData._currentGait != gaitData._nextGait) {
      createGait();
    }
    break;

  case 4:
    // Use NaturalGaitModification from FSM_State
    if (gaitData._currentGait != gaitData._nextGait) {
      createGait();
      period_time_natural = gaitData.periodTimeNominal;
      switching_phase_natural = gaitData.switchingPhaseNominal;
    } else {
      gaitData.periodTimeNominal = period_time_natural;
      gaitData.switchingPhaseNominal = switching_phase_natural;
      calcAuxiliaryGaitData();
    }
    break;
  }
}


/**
 * Creates the gait structure from the important defining parameters of each
 * gait
 *
 * To create a standard gait you should only need to define the following:
 *
 *   gaitData.periodTimeNominal
 *   gaitData.switchingPhaseNominal
 *   gaitData.phaseOffset
 *
 * The rest can be set to:
 *
 *   gaitData.gaitEnabled << 1, 1, 1, 1;
 *   gaitData.initialPhase = 0.0;
 *   gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
 *
 * These add flexibility to be used for very irregular gaits and transitions.
 */
template <typename T>
void GaitSchedulerPat<T>::createGait() {

  std::cout << "[GAIT] Transitioning gait from " << gaitData.gaitName
            << " to ";
  // Create the gait from the nominal initial parameters
  if (gaitData.NUM_FEET == 4) {
    // Pat gaits
    initializePatGait();

  } else if (gaitData.NUM_FEET == 2) {
    // Biped gaits
    initializeBipedGait();

  } else {
    std::cout << "[GAIT] initialize: Incorrect number of feet: " << gaitData.NUM_FEET << std::endl;
  }

  // Gait has switched
  gaitData._currentGait = gaitData._nextGait;

  std::cout << gaitData.gaitName << "\n" << std::endl;

  // Calculate the auxilliary gait information
  calcAuxiliaryGaitData();
}



template <typename T>
void GaitSchedulerPat<T>::calcAuxiliaryGaitData() {

  if (gaitData.overrideable == 1) {
    if (userParameters->gait_override == 2) {
      // gaitData.periodTimeNominal = userParameters->gait_period_time;
      // gaitData.switchingPhaseNominal = userParameters->gait_switching_phase;
    } else if (userParameters->gait_override == 4) {
      //gaitData.periodTimeNominal = userParameters->gait_period_time;
      //gaitData.switchingPhaseNominal = userParameters->gait_switching_phase;
    }
  }

  // Set the gait parameters for each foot
  for (int foot = 0; foot < gaitData.NUM_FEET; foot++) {
    if (gaitData.gaitEnabled(foot) == 1) {
      // The scaled period time for each foot
      gaitData.periodTime(foot) =
        gaitData.periodTimeNominal / gaitData.phaseScale(foot);

      // Phase at which to switch the foot from stance to swing
      gaitData.switchingPhase(foot) = gaitData.switchingPhaseNominal;

      // Initialize the phase variables according to offset
      gaitData.phaseVariable(foot) =
        gaitData.initialPhase + gaitData.phaseOffset(foot);

      // Find the total stance time over the gait cycle
      gaitData.timeStance(foot) =
        gaitData.periodTime(foot) * gaitData.switchingPhase(foot);

      // Find the total swing time over the gait cycle
      gaitData.timeSwing(foot) =
        gaitData.periodTime(foot) * (1.0 - gaitData.switchingPhase(foot));

    } else {
      // The scaled period time for each foot
      gaitData.periodTime(foot) = 100.0;

      // Phase at which to switch the foot from stance to swing
      gaitData.switchingPhase(foot) = 0.0;

      // Initialize the phase variables according to offset
      gaitData.phaseVariable(foot) = 0.0;

      // Foot is never in stance
      gaitData.timeStance(foot) = 0.0;

      // Foot is always in "swing"
      gaitData.timeSwing(foot) =
        gaitData.periodTime(foot) * (1.0 - gaitData.switchingPhase(foot));
    }
  }

}



/**
 * Prints relevant information about the gait and current gait state
 */
template <typename T>
void GaitSchedulerPat<T>::printGaitInfo() {
  // Increment printing iteration
  printIter++;

  // Print at requested frequency
  if (printIter == printNum) {
    std::cout << "[GAIT SCHEDULER] Printing Gait Info...\n";
    std::cout << "Gait Type: " << gaitData.gaitName << "\n";
    std::cout << "---------------------------------------------------------\n";
    if (gaitData.NUM_FEET == 4) {
      std::cout << "Enabled: " << gaitData.gaitEnabled(0) << " | "
                << gaitData.gaitEnabled(1) << " | " << gaitData.gaitEnabled(2)
                << " | " << gaitData.gaitEnabled(3) << "\n";
      std::cout << "Period Time: " << gaitData.periodTime(0) << "s | "
                << gaitData.periodTime(1) << "s | " << gaitData.periodTime(2)
                << "s | " << gaitData.periodTime(3) << "s\n";
      std::cout << "---------------------------------------------------------\n";
      std::cout << "Contact State: " << gaitData.contactStateScheduled(0) << " | "
                << gaitData.contactStateScheduled(1) << " | "
                << gaitData.contactStateScheduled(2) << " | "
                << gaitData.contactStateScheduled(3) << "\n";
      std::cout << "Phase Variable: " << gaitData.phaseVariable(0) << " | "
                << gaitData.phaseVariable(1) << " | " << gaitData.phaseVariable(2)
                << " | " << gaitData.phaseVariable(3) << "\n";
      std::cout << "Stance Time Remaining: " << gaitData.timeStanceRemaining(0)
                << "s | " << gaitData.timeStanceRemaining(1) << "s | "
                << gaitData.timeStanceRemaining(2) << "s | "
                << gaitData.timeStanceRemaining(3) << "s\n";
      std::cout << "Swing Time Remaining: " << gaitData.timeSwingRemaining(0)
                << "s | " << gaitData.timeSwingRemaining(1) << "s | "
                << gaitData.timeSwingRemaining(2) << "s | "
                << gaitData.timeSwingRemaining(3) << "s\n";
    } else if (gaitData.NUM_FEET == 2) {
      std::cout << "Enabled: " << gaitData.gaitEnabled(0) << " | "
                << gaitData.gaitEnabled(1) << "\n";
      std::cout << "Period Time: " << gaitData.periodTime(0) << "s | "
                << gaitData.periodTime(1) << "s\n";
      std::cout << "---------------------------------------------------------\n";
      std::cout << "Contact State: " << gaitData.contactStateScheduled(0) << " | "
                << gaitData.contactStateScheduled(1) << "\n";
      std::cout << "Phase Variable: " << gaitData.phaseVariable(0) << " | "
                << gaitData.phaseVariable(1) << "\n";
      std::cout << "Stance Time Remaining: " << gaitData.timeStanceRemaining(0)
                << "s | " << gaitData.timeStanceRemaining(1) << "s\n";
      std::cout << "Swing Time Remaining: " << gaitData.timeSwingRemaining(0)
                << "s | " << gaitData.timeSwingRemaining(1) << "s\n";

    }
    std::cout << std::endl;

    // Reset iteration counter
    printIter = 0;
  }
}


/**
 * Initializes the quadruped gaits
 */
template <typename T>
void GaitSchedulerPat<T>::initializePatGait() {
  // Case structure gets the appropriate parameters
  switch (gaitData._nextGait) {
    case GaitTypePat::STAND: {
    gaitData.gaitName = "STAND";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 10.0;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 1.0;
    gaitData.phaseOffset << 0.5, 0.5, 0.5, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 0;
    break;
  }

    case GaitTypePat::STAND_CYCLE: {
    gaitData.gaitName = "STAND_CYCLE";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 1.0;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 1.0;
    gaitData.phaseOffset << 0.5, 0.5, 0.5, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 0;
    break;
  }

    case GaitTypePat::STATIC_WALK: {
    gaitData.gaitName = "STATIC_WALK";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 1.25;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.8;
    gaitData.phaseOffset << 0.25, 0.0, 0.75, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::AMBLE: {
    gaitData.gaitName = "AMBLE";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.6250;
    gaitData.phaseOffset << 0.0, 0.5, 0.25, 0.75;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::TROT_WALK: {
    gaitData.gaitName = "TROT_WALK";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.6;
    gaitData.phaseOffset << 0.0, 0.5, 0.5, 0.0;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::TROT: {
    gaitData.gaitName = "TROT";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.5;
    gaitData.phaseOffset << 0.0, 0.5, 0.5, 0.0;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::TROT_RUN: {
    gaitData.gaitName = "TROT_RUN";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.4;
    gaitData.phaseOffset << 0.0, 0.5, 0.5, 0.0;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::PACE: {
    gaitData.gaitName = "PACE";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.35;
    gaitData.initialPhase = 0.25;
    gaitData.switchingPhaseNominal = 0.5;
    gaitData.phaseOffset << 0.0, 0.5, 0.0, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::BOUND: {
    gaitData.gaitName = "BOUND";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.4;
    gaitData.phaseOffset << 0.0, 0.0, 0.5, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::ROTARY_GALLOP: {
    gaitData.gaitName = "ROTARY_GALLOP";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.2;
    gaitData.phaseOffset << 0.0, 0.8571, 0.3571, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::TRAVERSE_GALLOP: {
    // TODO: find the right sequence, should be easy
    gaitData.gaitName = "TRAVERSE_GALLOP";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.2;
    gaitData.phaseOffset << 0.0, 0.8571, 0.3571, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::PRONK: {
    gaitData.gaitName = "PRONK";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.5;
    gaitData.phaseOffset << 0.0, 0.0, 0.0, 0.0;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::THREE_FOOT: {
    gaitData.gaitName = "THREE_FOOT";
    gaitData.gaitEnabled << 0, 1, 1, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.666;
    gaitData.phaseOffset << 0.0, 0.666, 0.0, 0.333;
    gaitData.phaseScale << 0.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::TWO_FOOT: {
    gaitData.gaitName = "TWO_FOOT";
    gaitData.gaitEnabled << 1, 0, 0, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.666;
    gaitData.phaseOffset << 0.0, 0.0, 0.0, 0.5;
    gaitData.phaseScale << 1.0, 0.0, 0.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::CUSTOM: {
    gaitData.gaitName = "CUSTOM";
    // TODO: get custom gait parameters from operator GUI
    break;
  }

    case GaitTypePat::TRANSITION_TO_STAND: {
    gaitData.gaitName = "TRANSITION_TO_STAND";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    T oldGaitPeriodTimeNominal = gaitData.periodTimeNominal;
    gaitData.periodTimeNominal = 3 * gaitData.periodTimeNominal;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal =
      (gaitData.periodTimeNominal + oldGaitPeriodTimeNominal
       * (gaitData.switchingPhaseNominal - 1)) / gaitData.periodTimeNominal;
    gaitData.phaseOffset << (gaitData.periodTimeNominal + oldGaitPeriodTimeNominal
                             * (gaitData.phaseVariable(0) - 1)) / gaitData.periodTimeNominal,
                             (gaitData.periodTimeNominal + oldGaitPeriodTimeNominal
                              * (gaitData.phaseVariable(1) - 1)) / gaitData.periodTimeNominal,
                             (gaitData.periodTimeNominal + oldGaitPeriodTimeNominal
                              * (gaitData.phaseVariable(2) - 1)) / gaitData.periodTimeNominal,
                             (gaitData.periodTimeNominal + oldGaitPeriodTimeNominal
                              * (gaitData.phaseVariable(3) - 1)) / gaitData.periodTimeNominal;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 0;
    break;
  }
  /*
     case GaitTypePat::TRANSITION_FROM_STAND:
        gaitData.gaitName = "TRANSITION_FROM_STAND";
        gaitData.gaitEnabled << 1, 1, 1, 1;


        gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;

        break;
  */
  default:
    // Go to stand
    gaitData.gaitName = "STAND";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 10.0;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 1.0;
    gaitData.phaseOffset << 0.5, 0.5, 0.5, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 0;

  }
}


/**
 * Initializes the biped gaits
 */
template <typename T>
void GaitSchedulerPat<T>::initializeBipedGait() {
  // Case structure gets the appropriate parameters
  switch (gaitData._nextGait) {
    case GaitTypePat::STAND: {
    gaitData.gaitName = "STAND";
    gaitData.gaitEnabled << 1, 1;
    gaitData.periodTimeNominal = 10.0;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 1.0;
    gaitData.phaseOffset << 0.5, 0.5;
    gaitData.phaseScale << 1.0, 1.0;
    gaitData.overrideable = 0;
    break;
  }

    case GaitTypePat::CUSTOM: {
    gaitData.gaitName = "CUSTOM";
    // TODO: get custom gait parameters from operator GUI
    break;
  }

    case GaitTypePat::WALK: {
    gaitData.gaitName = "WALK";
    gaitData.gaitEnabled << 1, 1;
    gaitData.periodTimeNominal = 0.75;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.667;
    gaitData.phaseOffset << 0.0, 0.5;
    gaitData.phaseScale << 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::RUN: {
    gaitData.gaitName = "RUN";
    gaitData.gaitEnabled << 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.4;
    gaitData.phaseOffset << 0.2, 0.7;
    gaitData.phaseScale << 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::HOP: {
    gaitData.gaitName = "HOP";
    gaitData.gaitEnabled << 1, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.3;
    gaitData.phaseOffset << 0.0, 0.0;
    gaitData.phaseScale << 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

  default:
    gaitData.gaitName = "STAND";
    gaitData.gaitEnabled << 1, 1;
    gaitData.periodTimeNominal = 10.0;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 1.0;
    gaitData.phaseOffset << 0.5, 0.5;
    gaitData.phaseScale << 1.0, 1.0;
    gaitData.overrideable = 0;
  }
}

/**
 * Prints relevant information about the gait and current gait state
 */
template <typename T>
void GaitSchedulerPat<T>::initializeBipedGait2() {
  // Case structure gets the appropriate parameters
  switch (gaitData._nextGait) {
    case GaitTypePat::STAND: {
    gaitData.gaitName = "STAND";
    gaitData.gaitEnabled << 0, 0, 1, 1;
    gaitData.periodTimeNominal = 10.0;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 1.0;
    gaitData.phaseOffset << 0.0, 0.0, 0.5, 0.5;
    gaitData.phaseScale << 0.0, 0.0, 1.0, 1.0;
    gaitData.overrideable = 0;
    break;
  }

    case GaitTypePat::CUSTOM: {
    gaitData.gaitName = "CUSTOM";
    // TODO: get custom gait parameters from operator GUI
    break;
  }

    case GaitTypePat::WALK: {
    gaitData.gaitName = "WALK";
    gaitData.gaitEnabled << 0, 0, 1, 1;
    gaitData.periodTimeNominal = 0.75;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.667;
    gaitData.phaseOffset << 0.0, 0.0, 0.0, 0.5;
    gaitData.phaseScale << 0.0, 0.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::RUN: {
    gaitData.gaitName = "RUN";
    gaitData.gaitEnabled << 0, 0, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.4;
    gaitData.phaseOffset << 0.0, 0.0, 0.2, 0.7;
    gaitData.phaseScale << 0.0, 0.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

    case GaitTypePat::HOP: {
    gaitData.gaitName = "HOP";
    gaitData.gaitEnabled << 0, 0, 1, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.3;
    gaitData.phaseOffset << 0.0, 0.0, 0.0, 0.0;
    gaitData.phaseScale << 0.0, 0.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;
  }

  default:
    gaitData.gaitName = "STAND";
    gaitData.gaitEnabled << 0, 0, 1, 1;
    gaitData.periodTimeNominal = 10.0;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 1.0;
    gaitData.phaseOffset << 0.0, 0.0, 0.5, 0.5;
    gaitData.phaseScale << 0.0, 0.0, 1.0, 1.0;
    gaitData.overrideable = 0;
  }
}

template class GaitSchedulerPat<double>;
template class GaitSchedulerPat<float>;
