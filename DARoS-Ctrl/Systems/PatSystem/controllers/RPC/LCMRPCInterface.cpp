#include "LCMRPCInterface.h"

LCMRPCInterface::LCMRPCInterface(ControlFSMData<float>* controlFSMData)
  : RPCInterface(controlFSMData) {

  std::cout << "[RPC] Initializing LCM Interface...\n";
  if (INITIALIZED == false)  {
    lcm->subscribe("CONTROLLER_rpc_outputs", &LCMRPCInterface::ReadRPCOutputsLCM, this);

    // Create the separate thread for solving the optimization
    RPC_CreateSolveThread();
    INITIALIZED = true;
  }
}


void LCMRPCInterface::RPC_CreateSolveThread() {
  // Create the RPC solving thread
  std::cout << "[RPC] Creating thread...\n";
  _solveThread = std::thread(&LCMRPCInterface::RPC_loopFunction, this);
}


void LCMRPCInterface::RPC_loopFunction() {
  RUN_RPC = true;
  while (RUN_RPC) {
    // Update the inputs to the RPC optimization
    SendRPCInputsLCM();

    usleep(2000);

    // Read the RPC optimization
    while (lcm->handleTimeout(0) != 0) {}
  }
}

void LCMRPCInterface::RPC_JoinThread() {
  _solveThread.join();
}

void LCMRPCInterface::ReadRPCOutputsLCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                        const rpc_outputs_lcmt *msg) {
  opt_time = msg->cpu_opt_time_microseconds;

  for (int k = 0; k < 5; k++) {
    dt_pred[k] = msg->dt_pred[k];

    for (int i = 0; i < 12; i++) {
      x_opt[k * 12 + i] = msg->x_opt[k * 12 + i];
    }

    for (int i = 0; i < 24; i++) {
      u_opt[k * 24 + i] = msg->u_opt[k * 24 + i];
    }

    for (int i = 0; i < 12; i++) {
      X_result[k * 36 + i] = x_opt[k * 12 + i];
    }

    for (int i = 0; i < 24; i++) {
      X_result[k * 36 + 12 + i] = u_opt[k * 24 + i];
    }
  }

  for (int i = 0; i < 12; i++) {
    p_opt[i] = msg->p_opt[i];
  }
}

