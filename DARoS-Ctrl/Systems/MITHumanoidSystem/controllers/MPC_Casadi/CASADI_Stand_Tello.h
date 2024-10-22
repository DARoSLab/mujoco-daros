#ifndef CASADI_Stand_Tello_H
#define CASADI_Stand_Tello_H
#include <auxillary/FootSwingTrajectory.h>

#include "cppTypes.h"
#include <ControlFSMData_Tello.h>
#include "params.hpp"
#include "optimizeStand.hpp"

class CASADI_Stand_Tello {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CASADI_Stand_Tello(
            float _dt, int _iterations_between_mpc, TelloParameters* parameters,
            const FloatingBaseModel<float> * model);


    const FloatingBaseModel<float> * _model;
    void run(ControlFSMData_Tello<float>& data);
    // For WBC ==================
    Vec3<float> pBody_des;
    Vec3<float> vBody_des;
    Vec3<float> aBody_des;

    Vec3<float> pBody_RPY_des;
    Vec3<float> vBody_Ori_des;

    Vec3<float> pFoot_des;
    Vec3<float> vFoot_des;
    Vec3<float> aFoot_des;

    Vec3<float> Fr_des[tello_contact::num_foot_contact];
    OptimizeStand opt;
    void init_casadi();

private:
    TelloParameters* _parameters = nullptr;
    float dt;
    float dtMPC;
    int iterationsBetweenMPC;
    int horizonLength;
    params p;
    int _iter = 0;


};





#endif 
