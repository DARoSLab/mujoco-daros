#ifndef OptimizeRLCommand_H
#define OptimizeRLCommand_H
#pragma once
#include <casadi/casadi.hpp>
#include "cppTypes.h"
#include "MPC_Casadi/eigenHelper.hpp"
#include <Dynamics/FloatingBaseModel.h>
#include <TelloParameters.h>

using namespace std;
using namespace casadi;
using namespace Eigen;


struct MPC_output
{
    MatrixXf x;
    MatrixXf xd;
    MatrixXf xw;
    MatrixXf xR;
    MatrixXf F_RToe;
    MatrixXf F_RHeel;
    MatrixXf F_LToe;
    MatrixXf F_LHeel;
    MatrixXf U;
    MatrixXf RPY;

};

struct State_MPC 
{
    DM x;
    DM xd;
    DM R;
    DM w;
    DM contactLoc;  //rightHeelLoc; rightToeLoc; leftHeelLoc; leftToeLoc;
    DM mpcTable;
};

class OptimizeRLCommand
{
    private:
        Opti opti;
        MX X;
        MX U;
        MX X0;
        MX Loc;
        MX Contact;
        MX XRef;
        MX Ref;
        MX cost;
        Slice all;
        string solverName;
        Dict p_opts;
        Dict s_opts; 
        int pred_hor;
        double m;
        DM gravity;
        DM mass_m;
        double dt; 
        double mu;



public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //constructor
    OptimizeRLCommand();

    void init(const FloatingBaseModel<float> * model, TelloParameters* p);

    MPC_output optimize(const FloatingBaseModel<float> * model, TelloParameters* p, State_MPC &S, VectorXf Ref, MatrixXf xRef);
};

#endif 
