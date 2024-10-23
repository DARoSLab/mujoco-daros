#ifndef LCM_COMMUNICATOR_H
#define LCM_COMMUNICATOR_H

#include <lcm-cpp.hpp>

#include <humanoid_state_info_lcmt.hpp>
#include <debug_visualization_lcmt.hpp>
#include <tello_control_lcmt.hpp>
#include <tello_joint_command_lcmt.hpp>
#include <tello_joint_data_lcmt.hpp>
#include <state_estimator_lcmt.hpp>

#include <thread>
#include <FBModel/FloatingBaseModel.h>
#include <SimUtilities/VisualizationData.h>
#include <Utilities/utilities.h>

template<typename T> class TelloSystem;

template<typename T>
class TelloLCMCommunicator{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TelloLCMCommunicator(TelloSystem<T>* sys, int ttl);
    ~TelloLCMCommunicator();

    void VisualInfoSending(FBModelState<T> _state, bool _is_estimate);
    void debugVisualInfoSending();
    void finalizeStep(tello_joint_data_lcmt* data,
        tello_joint_command_lcmt* command,
        state_estimator_lcmt* se);
  private:
    TelloSystem<T>* _tello_system;
    FloatingBaseModel<T> _model;
    VisualizationData* _visualizationData;

    lcm::LCM _tello_lcm;
    humanoid_state_info_lcmt _lcmt;
    debug_visualization_lcmt _debug_lcmt;

    void handleParameterLCM(const lcm::ReceiveBuffer *rbuf, 
        const std::string & chan, const tello_control_lcmt* msg);
    void _tello_LCMThread(){ while(true){ _tello_lcm.handle(); } }

    std::thread _tello_lcm_thread;
};
#endif
