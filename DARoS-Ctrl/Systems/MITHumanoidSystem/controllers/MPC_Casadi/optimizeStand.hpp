#ifndef optimizeStandTello_H
#define optimizeStandTello_H
#pragma once
#include <casadi/casadi.hpp>
#include "cppTypes.h"
#include "eigenHelper.hpp"
#include "params.hpp"
#include <FBModel/FloatingBaseModel.h>
#include <TelloParameters.h>

using namespace std;
using namespace casadi;
using namespace Eigen;


struct MPC_result
{
    DM x;
    DM xd;
    DM xw;
    DM xR;
    DM F_RToe;
    DM F_RHeel;
    DM F_LToe;
    DM F_LHeel;
    DM U;

};

struct State 
{
    DM x;
    DM xd;
    DM R;
    DM w;
    DM contactLoc;  //rightHeelLoc; rightToeLoc; leftHeelLoc; leftToeLoc;

};

class OptimizeStand
{
    private:
        Opti opti;
        MX X;
        MX U;
        MX X0;
        MX Loc;
        MX Contact;
        MX Ref;
        MX cost;
        Slice all;
        string solverName;
        Dict p_opts;
        Dict s_opts; 
        int pred_hor;
        double m;
        DM gravity;
        



public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //constructor
    OptimizeStand();

    void init(const FloatingBaseModel<float> * model, TelloParameters* p);

    MPC_result optimize(const FloatingBaseModel<float> * model, TelloParameters* p, State &S, VectorXf ref_traj, MatrixXd contact_seq);
};

#endif 
