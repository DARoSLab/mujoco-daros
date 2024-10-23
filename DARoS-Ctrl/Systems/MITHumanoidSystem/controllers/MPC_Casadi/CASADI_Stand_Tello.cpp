#include "CASADI_Stand_Tello.h"
#include <iostream>
#include <Utilities/Timer.h>
#include <pretty_print.h>
#include "eigenHelper.hpp"
#include <cppTypes.h>
#include "generate_contact_cycle.hpp"
////////////////////
// Controller
////////////////////

CASADI_Stand_Tello::CASADI_Stand_Tello(
    float _dt, int _iterations_between_mpc, TelloParameters* parameters,
    const FloatingBaseModel<float> * model) :
  _model(model),
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(2),
  dt(_dt),
  opt()
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  

}

void CASADI_Stand_Tello::init_casadi(){
  opt.init(_model, _parameters);
}



void CASADI_Stand_Tello::run(ControlFSMData_Tello<float>& data) {

  if (_iter % iterationsBetweenMPC == 0) {
    // NOTE: I THINK THIS R IS WORLD FRAME IN THE BODY FRAME NEED OT CHECK!!
    Eigen::Matrix3f R =  ori::quaternionToRotationMatrix(_model->_state.bodyOrientation);
    Eigen::Map<Eigen::Matrix<float, 9, 1>> R9by1(R.data());
    State s;
    s.x             = EigenVectorfTodm(_model->_state.bodyPosition);
    s.xd            = EigenVectorfTodm(_model->_state.bodyVelocity.tail(3));
    s.w             = EigenVectorfTodm(_model->_state.bodyVelocity.head(3));
    s.R             = EigenVectorfTodm(R9by1);
    s.contactLoc    = DM::zeros(3 * tello_contact::num_foot_contact);
    s.contactLoc(Slice(0, 3), 0) = EigenVectorfTodm(_model->_pGC[tello_contact::R_heel]);
    s.contactLoc(Slice(3, 6), 0) = EigenVectorfTodm(_model->_pGC[tello_contact::R_toe]);
    s.contactLoc(Slice(6, 9), 0) = EigenVectorfTodm(_model->_pGC[tello_contact::L_heel]);
    s.contactLoc(Slice(9, 12), 0) = EigenVectorfTodm(_model->_pGC[tello_contact::L_toe]);

    
    VectorXf cas_x_des =    _parameters->CAS_x_des;
    VectorXf cas_v_des =    _parameters->CAS_v_des;
    VectorXf cas_w_des =    _parameters->CAS_w_des;
    Vector3f CAS_RPY_des =  _parameters->CAS_RPY_des;
    Eigen::Matrix3f cas_R_des = ori::rpyToRotMat(CAS_RPY_des).transpose();
    //reshape to 9 by 1
    Eigen::Map<Eigen::Matrix<float, 9, 1>> cas_R_des_vec(cas_R_des.data());
    VectorXf ref_traj(18);
    ref_traj << cas_x_des, cas_v_des, cas_w_des, cas_R_des_vec;

    MatrixXd contact_seq = generate_contact_cycle(_parameters,_iter / iterationsBetweenMPC);

    MPC_result result = opt.optimize(_model, _parameters, s, ref_traj, contact_seq);
    DM F_RHeel  = result.F_RHeel(Slice(), 0);
    DM F_RToe   = result.F_RToe (Slice(), 0);
    DM F_LHeel  = result.F_LHeel(Slice(), 0);
    DM F_LToe   = result.F_LToe (Slice(), 0);
    


    // cout<<"F_RToe: \n"<<F_RToe<<endl;
    // cout<<"F_RHeel: \n"<<F_RHeel<<endl;
    // cout<<"F_LToe: \n"<<F_LToe<<endl;
    // cout<<"F_LHeel: \n"<<F_LHeel<<endl;

    DMat<float> mass_matrix = _model->getMassMatrix();


    //order matters
    Fr_des[0] = dmToEigenVectorf(F_RHeel);
    Fr_des[1] = dmToEigenVectorf(F_RToe);
    Fr_des[2] = dmToEigenVectorf(F_LHeel);
    Fr_des[3] = dmToEigenVectorf(F_LToe);

    pBody_des     = dmToEigenVectorf(result.x(Slice(), 1));
    vBody_des     = dmToEigenVectorf(result.xd(Slice(), 1));
    aBody_des     = Eigen::Vector3f::Zero(3);
    pBody_RPY_des = Eigen::Vector3f::Zero(3);
    vBody_Ori_des = Eigen::Vector3f::Zero(3);
    pFoot_des     = Eigen::Vector3f::Zero(3);
    vFoot_des     = Eigen::Vector3f::Zero(3);
    aFoot_des     = Eigen::Vector3f::Zero(3);
  }
  _iter ++;
}

