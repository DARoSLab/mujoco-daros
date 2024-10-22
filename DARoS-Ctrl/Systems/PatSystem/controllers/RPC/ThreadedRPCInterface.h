#ifndef THREADEDRPCINTERFACE_H
#define THREADEDRPCINTERFACE_H

// Contains all of the control related data
#include <state_machine/ControlFSMData.h>

#include <estimators/ContactEstimator.h>
#include <control/ctrl_utils/DesiredStateCommand.h>
#include <robots/PatBiped.h>

#include <SimUtilities/GamepadCommand.h>
#include <thread>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
//#include <eigen3/Eigen/Dense>

#include "rpc_inputs_lcmt.hpp"

#include "RegularizedPredictiveController.hpp"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "RPCInterface.h"

class ThreadedRPCInterface : public RPCInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static ThreadedRPCInterface* GetInstance(ControlFSMData<float> * _controlFSM){
    static ThreadedRPCInterface threaded_rpc_interface(_controlFSM);
    return &threaded_rpc_interface;
  }

  int testVal = 0;

  void UpdateInputsRPC();

  void SetFirstRun();
  void RPC_JoinThread();

private:
  unsigned long long iter_solve = 0;
  double elapsed_time = 0.;
  double average_computation_time = 0.;

  ThreadedRPCInterface(ControlFSMData<float>* _controlFSM);

  // IPOPT Solver
  void RPC_SolvePrediction();
  void RPC_SetIPOPTOptions();
  void RPC_CreateSolveThread();

  void ReadRPCSolution();

  void RPC_loopFunction();

  //rpc_inputs_lcmt input_data;
};

#endif
