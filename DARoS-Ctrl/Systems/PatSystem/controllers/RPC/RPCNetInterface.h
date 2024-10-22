#ifndef RPCNet_INTERFACE_H
#define RPCNet_INTERFACE_H

#include <state_machine/ControlFSMData.h>

#include <estimators/ContactEstimator.h>
#include <control/ctrl_utils/DesiredStateCommand.h>
//#include <controllers/auxillary/LegController.h>
#include <robots/PatBiped.h>

#include <thread>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>

#ifdef MACHINE_LEARNING_BUILD

#include "rpc_inputs_lcmt.hpp"
#include "RPCInterface.h"
#include <torch/script.h> // One-stop header.

class RPCNetInterface : public RPCInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static RPCNetInterface* GetInstance(ControlFSMData<float> * _controlFSM){
    static RPCNetInterface threaded_rpc_interface(_controlFSM);
    return &threaded_rpc_interface;
  }

  int testVal = 0;

  void UpdateInputsRPC();

  void SetFirstRun();
  void RPC_JoinThread();

private:
  RPCNetInterface(ControlFSMData<float>* _controlFSM);

  // IPOPT Solver
  void RPC_SolvePrediction();
  void RPC_CreateSolveThread();

  void ReadRPCSolution();

  void RPC_loopFunction();

  //rpc_inputs_lcmt input_data;

  static constexpr int dim_input = 9+12+3+1+1+1;
  float input_array[dim_input];
  torch::jit::script::Module module;

  std::vector<float> _mean_inputs;
  std::vector<float> _std_inputs;

  std::vector<float> _mean_outputs;
  std::vector<float> _std_outputs;
  std::vector<torch::jit::IValue> _torch_inputs;
};
#endif // #ifdef MACHINE_LEARNING
#endif
