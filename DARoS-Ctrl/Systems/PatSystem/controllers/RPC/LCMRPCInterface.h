#ifndef LCMRPCINTERFACE_H
#define LCMRPCINTERFACE_H

#include <thread>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>

// Contains all of the control related data
#include <state_machine/ControlFSMData.h>

#include "RPCInterface.h"

class LCMRPCInterface : public RPCInterface {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static LCMRPCInterface* GetInstance(ControlFSMData<float> * _controlFSM){
      static LCMRPCInterface lcm_rpc_interface(_controlFSM);
      return &lcm_rpc_interface;
    }

    void RPC_CreateSolveThread();
    void RPC_JoinThread();
    void RPC_loopFunction();
    void ReadRPCOutputsLCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                           const rpc_outputs_lcmt *msg);

  private:
    LCMRPCInterface(ControlFSMData<float>* _controlFSM);
};

#endif
