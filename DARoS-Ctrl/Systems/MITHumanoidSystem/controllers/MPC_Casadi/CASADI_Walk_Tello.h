#ifndef CASADI_Walk_Tello_H
#define CASADI_Walk_Tello_H
#include <auxillary/FootSwingTrajectory.h>

#include "cppTypes.h"
#include <ControlFSMData_Tello.h>
#include "optimizeStand.hpp"
#include "ControlFSMData_Tello.h"
#include "BipedGait.h"
#include "MPC_logger.hpp"
template <typename T>
class CASADI_Walk_Tello {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CASADI_Walk_Tello(
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
    OptimizeStand opt;
    void init_casadi();
    Vec4<float> contact_state;
    Vec4<float> swing_state;

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
    Eigen::MatrixXd _mpcTable;
    Vec3<float> pFoot_des_fixed[tello_contact::num_foot_contact];
    Vec3<float> pNextFoot[4];
    Vec3<float> pCurContact[4];
    MPC_logger mpcLogger;
    

};





#endif 
