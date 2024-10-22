#include "RPCNetInterface.h"
#include <Configuration.h>
#ifdef MACHINE_LEARNING_BUILD

#include <Utilities/cnpy.hpp>

RPCNetInterface::RPCNetInterface(ControlFSMData<float>* controlFSMData)
  : RPCInterface(controlFSMData) {

  std::cout << "[RPC] Initializing Interface...\n";

  std::string path = THIS_COM"systems/quadruped/NetData/RPC/";
  cnpy::NpyArray arr = cnpy::npy_load(path+"mean_inputs.npy");
  _mean_inputs = arr.as_vec<float>();

  arr = cnpy::npy_load(path+"std_inputs.npy");
  _std_inputs = arr.as_vec<float>();

  arr = cnpy::npy_load(path+"mean_outputs.npy");
  _mean_outputs = arr.as_vec<float>();

  arr = cnpy::npy_load(path+"std_outputs.npy");
  _std_outputs = arr.as_vec<float>();
  //std::cout<<_mean_inputs<<std::endl;
  //std::cout<<_std_inputs<<std::endl;
  //std::cout<<_mean_outputs<<std::endl;
  //std::cout<<_std_outputs<<std::endl;

  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(path + "traced_script_model.pt");
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
  }

  if (INITIALIZED == false)  {
    // Create the separate thread for solving the optimization
    RPC_CreateSolveThread();
    INITIALIZED = true;
  }
 
  printf("[RPC Net] Constructed\n");
}

void RPCNetInterface::RPC_CreateSolveThread() {
  // Create the RPC solving thread
  std::cout << "[RPC] Creating thread...\n";
  _solveThread = std::thread(&RPCNetInterface::RPC_loopFunction, this);
}


void RPCNetInterface::RPC_loopFunction() {
  RUN_RPC = true;
  while (RUN_RPC) {
    // Update the inputs to the RPC optimization
    UpdateInputsRPC();

    // Solve the RPC optimization
    RPC_SolvePrediction();

    // Read in the optimization solution
    ReadRPCSolution();
    //usleep(500);
  }
}


/*======================== Interface Functions ==========================*/
/**
 * Updates the optimization problem in the RPC Object for the upcoming
 * optimization.
 */
void RPCNetInterface::UpdateInputsRPC() {
  setRPCInputs();
  _torch_inputs.clear();

  int offset(0);
  input_array[offset + 0] = current_state[2]; // Height
  input_array[offset + 1] = current_state[3]; // Roll
  input_array[offset + 2] = current_state[4]; // Pitch
  offset = 3;
  for(size_t i(0); i<6; ++i){
    input_array[offset+i] = current_state[i + 6];
  }

  offset += 6;
  for(size_t i(0); i<12; ++i){
    //input_array[offset+i] = p_foot0[i];
    int foot = (int)i/3;
    input_array[offset+i] = (double)data->_stateEstimator->getLocalFootPos(foot)[i-3*foot];
  }
  offset += 12;
  // des vel
  input_array[offset+0] = x_desired[6];
  input_array[offset+1] = x_desired[7]; 
  input_array[offset+2] = x_desired[11]; 
  offset += 3;

  input_array[offset+0] = phase_variable[0];
  input_array[offset+1] = period_time[0];
  input_array[offset+2] = time_swing[0];


  //printf("dim: %d, %d\n", dim_input, _mean_inputs.size());
  //assert(dim_input == _mean_inputs.size());
  for(int i(0); i<dim_input; ++i){
    input_array[i] = (input_array[i] - _mean_inputs[i])/_std_inputs[i];
  }
  //for(int i(0); i<dim_input; ++i){
    //input_array[i] = _mean_inputs[i];
  //}

  _torch_inputs.push_back(torch::from_blob(input_array, {dim_input}));
/*
  //pretty_print(input_array, "input array", dim_input);
  //std::cout<<"torch input: "<<_torch_inputs<<std::endl;


  //_torch_inputs.push_back(torch::tensor({
        //-0.3592, -0.4189, -0.2297,  0.1289,  0.2063,  0.7126, -0.0550, -0.1257,
        //0.0080, -0.0545,  0.0397, -0.0320,  1.0869, -0.1619, -0.6998,  0.7355,
        //1.1641, -0.7206, -1.0769, -1.1187, -0.6571, -1.2688,  0.1547, -0.6600,
        //0.2173,  0.2227, -0.0147,  0.0225, -0.0294, -0.0294,  0.0225,  2.0402,
        //2.0402,  2.0402,  2.0402, -2.0375, -2.0375, -2.0375, -2.0375}));

  */
}


void RPCNetInterface::ReadRPCSolution() {
}

void RPCNetInterface::SetFirstRun() {
  FIRST_RUN = true;
}

void RPCNetInterface::RPC_JoinThread() {
  _solveThread.join();
}


void RPCNetInterface::RPC_SolvePrediction() {
    //std::cout<<"solve torch input: "<<_torch_inputs<<std::endl;
  //auto outputs = module.forward(_torch_inputs).toTuple();
  auto out = module.forward(_torch_inputs).toTensor();
    //std::cout<<"solve torch input after: "<<_torch_inputs<<std::endl;
  //at::Tensor out = outputs->elements()[1].toTensor();
  //at::Tensor out1 = outputs->elements()[0].toTensor();
    //std::cout <<"out: "<<out<<"\n";
    //std::cout <<"out1: "<<out1<<"\n";

  int out_idx(0);
  bool is_nan(false);
  if(isnan(out[0].item<float>())){
    is_nan = true;
    //std::cout<<_torch_inputs<<std::endl;
    std::cout <<"failed out: "<<out<<"\n";
    //pretty_print(u_opt, "u opt", 30);
    //pretty_print(p_opt, "p opt", 12);
    exit(0);
  }
  if(!is_nan){
    for(int foot(0); foot<4; ++foot){
      for(int axis(0); axis<3; ++axis){
        out_idx = 3*foot + axis;

        u_opt[foot*6 + 3 + axis] = 
          out[out_idx].item<float>()*_std_outputs[out_idx] + _mean_outputs[out_idx];
        p_opt[foot*3 + axis] = 
          out[12 + out_idx].item<float>()*_std_outputs[out_idx + 12] + 
          _mean_outputs[out_idx + 12];
      }
    }
    //pretty_print(p_opt, "p opt", 12);
    //pretty_print(_mean_outputs, "mean output");
    //p_opt[2] = 0.;
    //p_opt[5] = 0.;
    //p_opt[8] = 0.;
    //p_opt[11] = 0.;
  }  // is_nan
}

#endif
