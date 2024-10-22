#include "ThreadedRPCInterface.h"
#include <chrono>

static SmartPtr<RegularizedPredictiveController> RPCObj;
static SmartPtr<IpoptApplication> app;
static ApplicationReturnStatus status;

ThreadedRPCInterface::ThreadedRPCInterface(ControlFSMData<float>* controlFSMData)
  : RPCInterface(controlFSMData) {

  std::cout << "[RPC] Initializing Interface...\n";
  if (INITIALIZED == false)  {
    RPCObj = new RegularizedPredictiveController();

    // Initialize the IPOPT Application
    app = IpoptApplicationFactory();
    RPC_SetIPOPTOptions();

    // Create the separate thread for solving the optimization
    RPC_CreateSolveThread();
    INITIALIZED = true;
  }
}

void ThreadedRPCInterface::RPC_CreateSolveThread() {
  // Create the RPC solving thread
  std::cout << "[RPC] Creating thread...\n";
  _solveThread = std::thread(&ThreadedRPCInterface::RPC_loopFunction, this);
}


/*void ThreadedRPCInterface::RPC_CreateSendThread() {
  UpdateInputsRPC();
}*/


void ThreadedRPCInterface::RPC_loopFunction() {
  RUN_RPC = true;
  while (RUN_RPC) {
    // Update the inputs to the RPC optimization
    UpdateInputsRPC();

    // Solve the RPC optimization
    RPC_SolvePrediction();

    // Read in the optimization solution
    ReadRPCSolution();
  }
}


/*======================== Interface Functions ==========================*/
/**
 * Updates the optimization problem in the RPC Object for the upcoming
 * optimization.
 */
void ThreadedRPCInterface::UpdateInputsRPC() {
  setRPCInputs();

  // Update the Controller input information
  RPCObj->SetCurrentState(current_state);
  RPCObj->SetFootLocations(p_foot0);
  RPCObj->SetHipLocations(r_hip);
  RPCObj->SetDesiredTrajectory(x_desired);
  RPCObj->SetGaitParameters(gait_enabled, phase_variable, period_time, time_swing);
  RPCObj->SetRobotParameters(mass, inertia);
  RPCObj->SetEnvironmentParameters(gravity, mu, z_g);
  RPCObj->SetStateWeights(Q);
  RPCObj->SetInputRegularization(R);
  RPCObj->SetHeuristicGains(K_HX);

  //printf("===================== RPC Input Start ======================\n");
  //pretty_print(current_state, "curr state", 12);
  //pretty_print(p_foot0, "p foot 0", 12);
  //pretty_print(r_hip, "r hip", 12);
  //pretty_print(x_desired, "x_desired", 72);
  //pretty_print(gait_enabled, "gait enabled", 4);
  //pretty_print(phase_variable, "phase variable", 4);
  //pretty_print(period_time, "period time", 4);
  //pretty_print(time_swing, "time swing", 4);
  //printf("mass: %f\n", mass);
  //pretty_print(inertia, "inertia", 3);
  //pretty_print(gravity, "gravity", 3);
  //pretty_print(mu, "mu", 4);
  //pretty_print(z_g, "z_g", 4);
  //pretty_print(Q, "Q", 12);
  //pretty_print(R, "R", 24);
  //pretty_print(K_HX, "K HX", 30);
  //pretty_print(data->_desiredStateCommand->data.stateDes, std::cout, "DES command");
  //printf("===================== RPC Input End   ======================\n");
  //printf("\n");
  //RPCObj->SetTimeStart();
  //RPCObj->SetReady();

  // TEST (DH): For training
  SendRPCInputsLCM();
}


void ThreadedRPCInterface::ReadRPCSolution() {
  opt_time = 0.000001 * RPCObj->GetSolveTime();

  for (int k = 0; k < 5; k++) {
    dt_pred[k] = RPCObj->GetSolutionTime(k);

    for (int decision_var = 0; decision_var < 36; decision_var++) {
      X_result[k * 36 + decision_var] = RPCObj->GetSolution(k * 36 + decision_var);
    }

    for (int i = 0; i < 12; i++) {
      x_opt[k * 12 + i] = X_result[k * 36 + i];
    }

    for (int i = 0; i < 24; i++) {
      u_opt[k * 24 + i] = X_result[k * 36 + 12 + i];
    }
  }

  for (int i = 0; i < 12; i++) {
    p_opt[i] = RPCObj->GetStepLocation(i);
    //p_opt[i] = r_hip[i];
  }
  //pretty_print(dt_pred, "dt pred", 5);
  //pretty_print(X_result, "X result", 180);
  //pretty_print(x_opt, "x opt", 60);
  //pretty_print(u_opt, "u opt", 120);
  //pretty_print(p_opt, "p opt", 12);
}


/*
void ThreadedRPCInterface::PredictionCompensation(Vec12<float> xfb, double dt) {
  xfb1 = A*xfb + B*NonlinearInput(xfb, u, data->_gaitScheduler->gaitData.contactStateScheduled) + gravity_6D;
}
*/

void ThreadedRPCInterface::SetFirstRun() {
  FIRST_RUN = true;
}

void ThreadedRPCInterface::RPC_JoinThread() {
  _solveThread.join();
}


/*============================ IPOPT Solver =============================*/

void ThreadedRPCInterface::RPC_SetIPOPTOptions() {
  // TODO: Make this into a YAML file for easy changing
  // Printing Options
  app->Options()->SetIntegerValue("file_print_level", 0);
  app->Options()->SetIntegerValue("print_level", 0);
  app->Options()->SetIntegerValue("print_frequency_iter", 20);
  app->Options()->SetStringValue("print_timing_statistics", "yes");


  app->Options()->SetNumericValue("bound_frac", 1e-6);
  app->Options()->SetNumericValue("bound_push", 1e-6);

  // Stopping conditions and tolerances
  app->Options()->SetIntegerValue("max_iter", 50);
  app->Options()->SetNumericValue("max_cpu_time", 0.1);
  app->Options()->SetNumericValue("acceptable_tol", 0.00001);
  app->Options()->SetNumericValue("tol", 0.0001);

  app->Options()->SetStringValue("linear_solver", "ma27");
  //app->Options()->SetStringValue("mehrotra_algorithm","yes");
  app->Options()->SetStringValue("fixed_variable_treatment", "relax_bounds");
  //app->Options()->SetStringValue("honor_original_bounds","no")
  //app->Options()->SetStringValue("derivative_test", "first-order");
  //app->Options()->SetStringValue("derivative_test", "second-order");
  //app->Options()->SetNumericValue("derivative_test_tol",0.005);
  // Might actually be faster with hessian-approximation...
  //app->Options()->SetStringValue("hessian_approximation", "limited-memory");


  //app->Options()->SetStringValue("nlp_scaling_method","none");
  cout << "[RPC] Options Set" << endl;

}


void ThreadedRPCInterface::RPC_SolvePrediction() {
  // Microsecond delay for stability
  unsigned int microseconds = 10;

  // TODO: When to solve logic
  FIRST_RUN = true;
  if (FIRST_RUN == true) {
    if (RUN_RPC) {
      // Run the optimization
      auto start = std::chrono::system_clock::now();
      status = app->OptimizeTNLP(RPCObj);
      auto end = std::chrono::system_clock::now();
      elapsed_time = 
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

      average_computation_time += elapsed_time/600.;
      if(iter_solve%600 == 0){
        printf("[Threaded] average time taken: %f (ms)\n", average_computation_time);
        average_computation_time = 0.;
        //printf("%lu\n", iter_solve);
        std::cout<<iter_solve<<std::endl;
      }
    }
  } else {
    // Do nothing
  }
  iter_solve++;
}
