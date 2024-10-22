#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

#include "EstimatorParameters.h"
#include <controllers/auxillary/TelloJointController.h>

#include <Dynamics/Robot.h>
#include <cppTypes.h>
#include <SimUtilities/VisualizationData.h>
#include "state_estimator_lcmt.hpp"
/*!
 * Result of state estimation
 */
template <typename T>
struct StateEstimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<T> contactEstimate;
  DiVec contactState;

  Vec3<T> position;
  Vec3<T> vBody;
  Quat<T> orientation;
  Vec3<T> omegaBody;
  RotMat<T> rBody;   // rotation matrix, this is orientation of world frame in the body frame: b_R_w
  Vec3<T> rpy;

  Vec3<T> omegaWorld;
  Vec3<T> vWorld;
  Vec3<T> aBody, aWorld;
};

/*!
 * Inputs for state estimation.
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also a setter method to
 * StateEstimatorContainer)
 */
template <typename T>
struct StateEstimatorData {
  StateEstimate<T> result;  // where to write the output to
  Vec4<T> contactPhase;
  Vec4<T> footHeights;
  Vec12<T> contactForces;

  const IMU_Data* imuData;
  const CheaterState* cheaterState;
  JointControlData<T>* const * jointData;
  const Robot<T>* robot;
  const EstimatorParameters* parameters;
};

/*!
 * All Estimators should inherit from this class
 */
template <typename T>
class GenericEstimator {
public:
  virtual void run() = 0;
  virtual void setup() = 0;

  void setData(StateEstimatorData<T> * data) { _stateEstimatorData = data; }

  virtual ~GenericEstimator() = default;
  StateEstimatorData<T> * _stateEstimatorData;
};

/*!
 * Main State Estimator Class
 * Contains all GenericEstimators, and can run them
 * Also updates visualizations
 */
template <typename T>
class StateEstimatorContainer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Construct a new state estimator container (uses localization data)
   */
StateEstimatorContainer(CheaterState* cheaterState,
    IMU_Data* imuData,
    JointControlData<T>** jointData,
    EstimatorParameters* parameters,
    Robot<T>* robot);
  /*!
   * Run all estimators
   */
  void run();

  /*!
   * Get the result
   */
  const StateEstimate<T>& getResult() { return _data.result; }

  /*!
   * Set the contact phase
   */
  void setContactPhase(const Vec4<T>& phase) {
    _data.contactPhase = phase;
  }

  /*!
  * Set the contact forces
  */
  void setContactForces(const Vec12<T>& contactForces) {
    _data.contactForces = contactForces;
  }

  /*!
  * Set the foot heights
  */
  void setFootHeights(const Vec4<T>& footHeights) {
    _data.footHeights = footHeights;
  }

  /*!
  * Set the lcm data
  */
  void setLcm(state_estimator_lcmt* lcm_data) {
    for (int i = 0; i < 3; i++) {
      lcm_data->p[i] = _data.result.position[i];
      lcm_data->vWorld[i] = _data.result.vWorld[i];
      lcm_data->vBody[i] = _data.result.vBody[i];
      lcm_data->rpy[i] = _data.result.rpy[i];
      lcm_data->omegaBody[i] = _data.result.omegaBody[i];
      lcm_data->omegaWorld[i] = _data.result.omegaWorld[i];
      lcm_data->aBody[i] = _data.result.aBody[i];
      lcm_data->aWorld[i] = _data.result.aWorld[i];
    }

    for (int i = 0; i < 4; i++) {
      lcm_data->quat[i] = _data.result.orientation[i];
      lcm_data->contact_estimate[i] = _data.result.contactEstimate[i];
    }

    unsigned long milliseconds_since_epoch = 
      std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::system_clock::now().time_since_epoch()).count();
    lcm_data->lcm_published_timestamp = milliseconds_since_epoch;

  }

  /*!
   * Add an estimator of the given type
   * @tparam EstimatorToAdd
   */
  template <typename EstimatorToAdd>
  void addEstimator() {
    auto* estimator = new EstimatorToAdd();
    estimator->setData(&_data);
    estimator->setup();
    _estimators.push_back(estimator);
  }

  /*!
   * Remove all estimators of a given type
   * @tparam EstimatorToRemove
   */
  template <typename EstimatorToRemove>
  void removeEstimator() {
    int nRemoved = 0;
    _estimators.erase(
      std::remove_if(_estimators.begin(), _estimators.end(),
    [&nRemoved](GenericEstimator<T>* e) {
      if (dynamic_cast<EstimatorToRemove*>(e)) {
        delete e;
        nRemoved++;
        return true;
      } else {
        return false;
      }
    }),
    _estimators.end());
  }

  /*!
   * Remove all estimators
   */
  void removeAllEstimators() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
    _estimators.clear();
  }

  ~StateEstimatorContainer() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
  }

private:
  StateEstimatorData<T> _data;
  std::vector<GenericEstimator<T>*> _estimators;
  Vec4<T> _phase;
  Vec4<T> _footHeights;
};

#endif  // PROJECT_STATEESTIMATOR_H
