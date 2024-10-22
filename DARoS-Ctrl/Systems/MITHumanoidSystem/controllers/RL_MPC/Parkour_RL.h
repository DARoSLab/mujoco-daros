#ifndef PARKOUR_Tello_H
#define PARKOUR_Tello_H
#include <auxillary/FootSwingTrajectory.h>

#include "cppTypes.h"
#include <ControlFSMData_Tello.h>
#include "MPC_Casadi/optimizeStand.hpp"
#include "optimizeRLCommand.hpp"
#include "ControlFSMData_Tello.h"
#include "MPC_Casadi/BipedGait.h"
#include "RL2dAgent_.hpp"
#include "gaitManagerRL.hpp"
#include "terrainAnimator.hpp"
using namespace std;


template <typename T>
class Parkour_RL {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Parkour_RL(
            float _dt, int _iterations_between_mpc, TelloParameters* parameters,
            FloatingBaseModel<T> * model, ControlFSMData_Tello<T>* _fsm_data);

    FloatingBaseModel<T> * _model;
    void run(ControlFSMData_Tello<T>& data);
    // For WBC ==================
    Vec3<float> pBody_des;
    Vec3<float> vBody_des;
    Vec3<float> aBody_des;

    Vec3<float> pBody_RPY_des;
    Vec3<float> vBody_Ori_des;

    Vec3<float> pFoot_des[tello_contact::num_foot_contact];
    Vec3<float> vFoot_des[tello_contact::num_foot_contact];
    Vec3<float> aFoot_des[tello_contact::num_foot_contact];

    Vec3<float> Fr_des[tello_contact::num_foot_contact];
    OptimizeRLCommand optRL;
    void init_casadi();
    Vec4<float> contact_state;
    Eigen::VectorXd terrain;

private:
    TelloParameters* p = nullptr;
    ControlFSMData_Tello<T>* _fsm_data;
    float dt;
    float dtMPC;
    int iterationsBetweenMPC;
    int pred_hor;
    int _iter = 0;
    bool firstSwing[tello_contact::num_foot_contact];
    FootSwingTrajectory<float> footSwingTrajectories[tello_contact::num_foot_contact];
    OffsetDurationGait* walking;
    float swingTimes[tello_contact::num_foot_contact];
    float swingTimeRemaining[tello_contact::num_foot_contact];
    Vec3<float> pFoot_des_fixed[tello_contact::num_foot_contact];
    Vec3<float> pNextFoot[4];
    Vec3<float> pCurContact[4];
    RL2dAgent_ agent;
    
    void generate_terrain();
    void add_pit(int start, int len);
    void add_stair(int start);
    int countdownRL = 0;
    VectorXd get_terrain_obs(double x_loc);
    double get_contact_y(double x_loc);
    MatrixXf get_desireX(double desired_vel, float x);

    State2d s2d;
    gaitManagerRL* gaitManager;
    double desired_vel;
    /*
    MPC_State has 3 states: 
    2, means we need start from a brand new RL action
    1, means we need a RL input based on the preidiciton of the current MPC
    0, means we can just follow the current RL command,
    */
    int MPC_State = 2;
    bool leftFootSwingUp[2]={false, false};
    terrainAnimator animator;
    double terrain_density = 0.05;
    int rough_terrain_start;
    int rough_terrain_end;
};





#endif 
