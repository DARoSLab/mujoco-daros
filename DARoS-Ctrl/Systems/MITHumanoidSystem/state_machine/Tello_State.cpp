/*============================= FSM State =============================*/
#include "Tello_State.h"

/**
 * Constructor for the FSM State class.
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name
 * @param stateStringIn the string name of the current FSM state
 */
template <typename T>
Tello_State<T>::Tello_State(ControlFSMData_Tello<T> *_controlFSMData,
    Tello_StateName stateNameIn, std::string stateStringIn):
  _fsm_data(_controlFSMData),
  stateName(stateNameIn),
  stateString(stateStringIn),
  _model(_controlFSMData->_tello_model) {
    // _model = _controlFSMData->_tello_model;
    // this->_system->getRobot()->buildModel(_sim_dyn_model);
    _state.q = DVec<T>::Zero(_model->_nDof-6);
    _state.qd = DVec<T>::Zero(_model->_nDof-6);

    transitionData.zero();
    std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn << std::endl;
  }


/* =================== Robot Utilities =================== */
/**
 * Joint PD control for a given limb.
 *
 * @param limb the limb number to control
 * @param qDes desired joint position
 * @param dqDes desired joint velocity
 */
template <typename T>
void Tello_State<T>::jointPDControl(int idx, DVec<T> qDes, DVec<T> qdDes, DMat<T> kpMat,DMat<T> kdMat) {
  _fsm_data->_jointController->_commands[idx]->kpJoint = kpMat;
  _fsm_data->_jointController->_commands[idx]->kdJoint = kdMat;
  _fsm_data->_jointController->_commands[idx]->tauFeedForward.setZero();
  _fsm_data->_jointController->_commands[idx]->qDes = qDes;
  _fsm_data->_jointController->_commands[idx]->qdDes = qdDes;
}

/**
 * Current joint position of tello.
 *
 * returns joint positions as DVec
 */
template <typename T>
DVec<T>  Tello_State<T>::get_current_jpos() {
    DVec<T> jpos = DVec<T>::Zero(tello::num_act_joint);
    int joint_count = 0;
    for(size_t idx(0); idx < tello::num_joint_group; ++idx){
        for(int jidx(0); jidx < _fsm_data->_jointController->_datas[idx]->_num_joints;++jidx){
            jpos[joint_count] = _fsm_data->_jointController->_datas[idx]->q[jidx];
            joint_count++;
        }
    }
    return jpos;
}


/* =================== Safety Functions =================== */
/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
template <typename T>
void Tello_State<T>::turnOnAllSafetyChecks() {
  // Pre controls safety checks
  checkSafeOrientation = true;  // check roll and pitch

  // Post control safety checks
  checkPDesFoot = true;          // do not command footsetps too far
  checkForceFeedForward = true;  // do not command huge forces
  checkLegSingularity = true;    // do not let leg
}


/**
 *
 */
template <typename T>
void Tello_State<T>::turnOffAllSafetyChecks() {
  // Pre controls safety checks
  checkSafeOrientation = false;  // check roll and pitch

  // Post control safety checks
  checkPDesFoot = false;          // do not command footsetps too far
  checkForceFeedForward = false;  // do not command huge forces
  checkLegSingularity = false;    // do not let leg
}


template<typename T>
bool Tello_State<T>::locomotionSafe() {
  //auto &seResult = _data->_stateEstimator->getResult();

  //const T max_roll = 40;
  //const T max_pitch = 50;

  //if (std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    //printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    //return false;
  //}

  //if (std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    //printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    //return false;
  //}
  return true;
}

template<typename T>
void Tello_State<T>::UpdateModel(){
  StateEstimate<T> state_est = _fsm_data->_stateEstimator->getResult();

  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i+3] = state_est.vBody[i];
  }

  int joint_count = 0;
  for(size_t group(0); group < tello::num_joint_group; ++group){
    for(int jidx(0); jidx < _fsm_data->_jointController->_datas[group]->_num_joints; ++jidx){
      _state.q[joint_count] = _fsm_data->_jointController->_datas[group]->q[jidx];
      _state.qd[joint_count] = _fsm_data->_jointController->_datas[group]->qd[jidx];
      joint_count++;
    }
  }

  // pretty_print(_state.bodyPosition, std::cout, "body position");
  // pretty_print(_state.bodyOrientation, std::cout, "body orientation");
  // pretty_print(_state.bodyVelocity, std::cout, "body velocity");
  // pretty_print(_state.q, std::cout, "joint orientations");
  // pretty_print(_state.qd, std::cout, "joint velocities");

  _model->setState(_state);
  _model->contactJacobians();
  _model->FullcontactJacobians();
  _model->massMatrix();//Comment if using massandCoriolisMatrix()
  _model->generalizedGravityForce();
  _model->generalizedCoriolisForce();
  _model->centroidMomentumMatrix();
  _model->massandCoriolisMatrix();//Comment if not needed
  //DMat<T> A = _model->getMassMatrix();
  //std::cout<<A.block(0,0, 6,6)<<std::endl;

}

/**
 * Visualize heels and toes.
 */
template <typename T>
void  Tello_State<T>::viz_heel_toe() {

    for (size_t i(0); i<_model->_pGC.size(); i++){
        auto* currSphere = _fsm_data->_visualizationData->addSphere();
        currSphere->color = {0.95, 0.05, 0.05, 0.7}; // red = desired foot pos, contact scheduled
        currSphere->radius = 0.05;
        currSphere->position = _model->_pGC[i];
        //pretty_print(_model->_pGC[i], std::cout, "pGC");
    }
}


template class Tello_State<float>;
