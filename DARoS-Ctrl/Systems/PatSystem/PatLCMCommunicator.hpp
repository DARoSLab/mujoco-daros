#ifndef Pat_LCM_COMMUNICATOR_H
#define Pat_LCM_COMMUNICATOR_H

#include <lcm-cpp.hpp>
#include <quadruped_parameters_lcmt.hpp>
#include <quadruped_menu_data_lcmt.hpp>
#include <pat_state_info_lcmt.hpp>
#include <debug_visualization_lcmt.hpp>
#include <pat_leg_control_command_lcmt.hpp>
#include <pat_leg_control_data_lcmt.hpp>
#include <state_estimator_lcmt.hpp>
#include <thread>
#include <FBModel/FloatingBaseModel.h>
#include <SimUtilities/VisualizationData.h>
#include <Utilities/utilities.h>

template<typename T> class PatSystem;
template<typename T> class System;

template<typename T>
class PatLCMCommunicator{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PatLCMCommunicator(PatSystem<T> * system, int ttl);
    ~PatLCMCommunicator();
    void VisualInfoSendingURDF(const FBModelState<T> & _state, bool _is_estimate);
    void debugVisualInfoSending();
    void finalizeStep(pat_leg_control_data_lcmt* data,
        pat_leg_control_command_lcmt* command,
        state_estimator_lcmt* se);


  private:
    PatSystem<T>* _pat_system;
    FloatingBaseModel<T> _dyn_model;
    VisualizationData* _visualizationData;
    lcm::LCM _pat_lcm;
    pat_state_info_lcmt _lcmt;
    debug_visualization_lcmt _debug_lcmt;

    void handleParameterLCM(const lcm::ReceiveBuffer *rbuf,
        const std::string & chan, const quadruped_parameters_lcmt * msg);
    void handleMenuDataLCM(const lcm::ReceiveBuffer *rbuf,
        const std::string & chan, const quadruped_menu_data_lcmt * msg);
    void _pat_LCMThread(){ while(true){ _pat_lcm.handle(); } }

    std::thread _pat_lcm_thread;
};
#endif
