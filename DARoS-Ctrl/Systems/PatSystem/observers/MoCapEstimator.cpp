/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */
#ifdef MOCAP_BUILD
#include "MoCapEstimator.h"
#include <unistd.h>
#include <mutex>
#include <pretty_print.h>
std::mutex mtx;
template <typename T>
void MoCapEstimator<T>::setup() {
  if(owl.open(address) <= 0 || owl.initialize() <= 0){

    std::cerr << "Couldn't Open Mocap" << '\n';
    return;

  }

  uint32_t tracker_id = 0;
  owl.createTracker(tracker_id, "rigid", "myrigid");
  owl.assignMarker(tracker_id, 3, "4", "pos=0,920,-7.615");
  owl.assignMarker(tracker_id, 5, "5", "pos=-7.615,795,0");
  owl.assignMarker(tracker_id, 7, "7", "pos=-7.615,545,0");
  _body_position_est.setZero();
  _body_velocity_est.setZero();
  _body_orientation_est.setZero();
  // start streaming
  std::cout << "[INIT MoCap]" << '\n';
  owl.streaming(1);
  std::thread(&MoCapEstimator<T>::_pat_mocap_thread, this);
  // _mc_thread.detach();
}

template <typename T>
MoCapEstimator<T>::MoCapEstimator() {
}
template <typename T>
void MoCapEstimator<T>::run(){
  mtx.lock();
  this->_stateEstimatorData->>result.position << _body_position_est;
  this->_stateEstimatorData->>result.vWorld << _body_velocity_est;
  this->_stateEstimatorData->>result.vBody =
    this->_stateEstimatorData->>result.rBody *
    this->_stateEstimatorData->>result.vWorld;
  mtx.unlock();
}
/*!
 * Run state estimator
 */
template <typename T>
void MoCapEstimator<T>::_pat_mocap_thread() {
  while(1)
  {
      // std::cout << "/* message */" << '\n';
      const OWL::Event *event = owl.nextEvent(TIMEOUT);
      if(!event) return;

      if(event->type_id() == OWL::Type::ERROR)
        {
          std::cerr << event->name() << ": " << event->str() << std::endl;
          return;
        }
      else if(event->type_id() == OWL::Type::FRAME)
        {
          // std::cout << "frame " << '\n';
          if(event->find("rigids", rigids) > 0)
            {
              for(OWL::Rigids::iterator r = rigids.begin(); r != rigids.end(); r++){

                if(r->cond > 0){
                  // _curr_time = chrono::steady_clock::now();
                  mtx.lock();
                  _body_position_est[0] = r->pose[0]/1000.0;
                  _body_position_est[1] = -r->pose[2]/1000.0;
                  _body_position_est[2] = r->pose[1]/1000.0 - Z_OFFSET;
                  if(first_visit){
                    _body_velocity_est.setZero();
                    first_visit = false;
                  }else{
                    // auto dt = 0.000001* chrono::duration_cast<chrono::microseconds>(
                    //       _curr_time-_prev_time).count();
                    _body_velocity_est = 250*(_body_position_est - _prev_body_position_est);
                  }
                  mtx.unlock();
                  _prev_body_position_est << _body_position_est;
                  _body_orientation_est << r->pose[3], r->pose[4], r->pose[5], r->pose[6];
                  // std::cout << "bp: " << _body_position_est << '\n';

                  /*
                  for(int i(0); i<3; ++i)
                    pat_est_data.body_pos[i] = _body_position_est[i];
                  for(int i(0); i<4; ++i)
                    pat_est_data.body_ori_quat_est[i] = r->pose[3+i];
                  */

                  // for(int i(0); i<3; ++i){
                  //   pat_mocap_data.bodyPosition[i] = _body_position_est[i];
                  //   pat_mocap_data.bodyVelocity[i] = _body_velocity_est[i];
                  // }
                  // for(int i(0); i<4; ++i)
                  //   pat_mocap_data.bodyOrientation[i] = r->pose[3+i];

                  //lc.publish("pat_state_est_info", &pat_est_data);
                  // lc.publish("pat_mocap_est_info", &pat_mocap_data);
                  // _prev_time = _curr_time;
                }
              }
            }
        }
        usleep(1000);
    } // while

}

template class MoCapEstimator<float>;
template class MoCapEstimator<double>;


/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterMoCapEstimator<T>::run() {
  this->_stateEstimatorData->>result.position = this->_stateEstimatorData->>cheaterState->position.template cast<T>();
  this->_stateEstimatorData->>result.vWorld =
    this->_stateEstimatorData->>result.rBody.transpose().template cast<T>() * this->_stateEstimatorData->>cheaterState->vBody.template cast<T>();
  this->_stateEstimatorData->>result.vBody = this->_stateEstimatorData->>cheaterState->vBody.template cast<T>();
}

template class CheaterMoCapEstimator<float>;
template class CheaterMoCapEstimator<double>;
#endif // MOCAP_BUILD
