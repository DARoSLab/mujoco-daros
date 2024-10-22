#include "RL2dAgent_.hpp"
#include <torch/script.h> 
#include <ParamHandler/ParamHandler.hpp>

torch::jit::script::Module module;

RL2dAgent_::RL2dAgent_(){
    // store an address in a pointer
    ParamHandler param(THIS_COM"Systems/loco-jump/configuration.yaml");
    std::string model_path;
    param.getString("model_path", model_path);
    
    //concatenate THIS_COM and model_path
    model_path = THIS_COM + model_path;
    std::cout<< "model_path: "<<model_path<<std::endl;
    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        module = torch::jit::load(model_path);
    }
    catch (const c10::Error& e) {
        std::cerr << "error loading the model\n";
    }
}

VectorXd RL2dAgent_::post_process(VectorXd &action) {
    VectorXd post_action = VectorXd::Zero(action.size());
    // post_action(0) = action(0) * 0.5  + 0.7;
    // post_action(1) = action(1) * 0.5  + 0.5;
    // post_action(2) = action(2) * 0.025 + 0.175;
    // post_action(3) = action(3) * 0.05 + 0.35;
    // post_action(4) = action(4) * 0.25 + 0.25;
    // post_action(5) = action(5) * 0.05 + 0.15;
    // post_action(6) = action(6) * 0.1 + 0.3 ;
    post_action(0) = action(0)  * 0.5  + 0.6;  // desired_vel   range from 0.1 to 1.1
    post_action(1) = action(1) * 0.4  + 0.4;
    post_action(2) = action(2) * 0.2 + 0.2;
    post_action(3) = action(3);
    return post_action;
}

VectorXd RL2dAgent_::module_forward(State2d &s){

    //1. pushing state to the inputs
    VectorXd obs_d = VectorXd::Zero(26);
    
    obs_d << s.x - s.curr_contact_loc, s.xd, s.theta, s.w, s.terrain_obs;
    // cout<<"obs_d: "<<obs_d<<endl;
    
    //2. convert to float 
    VectorXf obs_f = VectorXf::Zero(obs_d.size()); 
    for (int i = 0; i < obs_d.size(); ++i) {
        obs_f(i) = static_cast<float>(obs_d(i));  // Convert and assign values
    }
    // cout<<"obs_f: "<<obs_f<<endl;

    //3. convert to tensor
    std::vector<int64_t> dims = {obs_f.cols(),obs_f.rows()};
    torch::Tensor obs = torch::from_blob(obs_f.data(), dims).clone();
    
    //4. push to the inputs
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(obs);


    //5. forward
    at::Tensor output = module.forward(inputs).toTensor();

    //6. return the output
    VectorXd action = VectorXd::Zero(4);

    for (int i = 0; i < action.size(); ++i) {
        action(i) = output[0][i].item<double>();  // Convert and assign values
    }

    VectorXd post_action = post_process(action);

    return post_action;
    
}

