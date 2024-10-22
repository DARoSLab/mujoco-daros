#include "CASADI_Walk_Tello.h"
#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/pretty_print.h>
#include "eigenHelper.hpp"
#include <cppTypes.h>
#include "generate_contact_cycle.hpp"
////////////////////
// Controller
////////////////////
template <typename T>
CASADI_Walk_Tello<T>::CASADI_Walk_Tello(
    float _dt, int _iterations_between_mpc, TelloParameters* parameters,
     FloatingBaseModel<T> * model, ControlFSMData_Tello<T>* _fsm_data) :
  _model(model),
  _fsm_data(_fsm_data),
  iterationsBetweenMPC(_iterations_between_mpc),
  pred_hor(parameters->CAS_pred_hor),
  dt(_dt),
  opt()
{
  p = parameters;
  Vec4<int> walking_offsets; walking_offsets << p->gait_right_offset,
                                                p->gait_right_offset,
                                                p->gait_left_offset,
                                                p->gait_left_offset;
  Vec4<int> walking_durations; walking_durations << p->gait_duration,
                                                    p->gait_duration,
                                                    p->gait_duration,
                                                    p->gait_duration;
  walking = new OffsetDurationGait(p->gait_num_segments, walking_offsets, walking_durations,"Walking");
  // std::cout << "MPC Table: " << std::endl;
  int* mpcTable = walking->getMpcTable();

  for(int i = 0; i < p->gait_num_segments; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      std::cout << mpcTable[i*4 + j] << " ";
    }
    std::cout << std::endl;
  }

  dtMPC = dt * iterationsBetweenMPC;

  for(int i = 0; i < tello_contact::num_foot_contact; i++)
  {
    firstSwing[i] = true;
    pFoot_des[i].setZero();
    vFoot_des[i].setZero();
    aFoot_des[i].setZero();
  }

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
  _mpcTable = Eigen::MatrixXd::Zero(tello_contact::num_foot_contact, p->CAS_pred_hor);

}
template <typename T>
void CASADI_Walk_Tello<T>::init_casadi(){
  opt.init(_model, p);

}

template <typename T>
void CASADI_Walk_Tello<T>::run(ControlFSMData_Tello<T>& data) 
{

  walking->setIterations(iterationsBetweenMPC, _iter);

  auto& seResult = data._stateEstimator->getResult();
  contact_state = walking->getContactState();
  swing_state  = walking->getSwingState();
  // std::cout<< "contact_state: \n" << contact_state << std::endl;
  // std::cout<< "swing_state: \n" << swing_state << std::endl;

  for (int i = 0; i <tello_contact::num_foot_contact; i++)
  {
    swingTimes[i] = walking->getCurrentSwingTime(dtMPC, i);
    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } 
    else {
      swingTimeRemaining[i] -= dt;
    }
    footSwingTrajectories[i].setHeight(p->gait_swing_height); 

    Vec3<float> des_vel;

    des_vel[0] = p->CAS_v_des[0];
    des_vel[1] = p->CAS_v_des[1];
    des_vel[2] = 0.0;
    // std::cout<<"des_vel: \n"<< des_vel << std::endl;
    Vec3<float> legOffSet (
      (i ==0 || i == 2 ) ? p->gait_heel_offset : p->gait_toe_offset,
      (i <=1) ? -p->gait_foot_offset : p->gait_foot_offset,
      0.0
    );

    // this is bacially the position of the hip in the world frame after the swing
    Vec3<T> pf = seResult.position + seResult.rBody.transpose() * (legOffSet + des_vel * swingTimeRemaining[i]);
    float stance_time = walking->getCurrentStanceTime(dtMPC, i);
    pf[0] += seResult.vBody[0] * .5 * stance_time + p->gait_vel_gain * (seResult.vBody[0] - des_vel[0]);
    pf[1] += seResult.vBody[1] * .5 * stance_time + p->gait_vel_gain * (seResult.vBody[1] - des_vel[1]);
    pf[2] = -0.0001;
    footSwingTrajectories[i].setFinalPosition(pf);
    pNextFoot[i] = pf;
    pCurContact[i] =  _model->_pGC[i+tello_contact::R_heel];
    // std::cout<<"Swing_state \n "<< swing_state << std::endl;
    // std::cout<<"contact_state \n "<< contact_state << std::endl;
    if (swing_state[i] > 0) //foot is in swing
    {
      // std::cout<<"Swing final position \n "<< pf << std::endl;
      // exit(0);
      if(firstSwing[i])
      {  //if the foot just started swinging
        firstSwing[i] = false;
        footSwingTrajectories[i].setInitialPosition(pCurContact[i]); //record where the foot started
      }
      footSwingTrajectories[i].computeSwingTrajectoryBezier(swing_state[i], swingTimes[i]); //compute the swing trajectory
      pFoot_des[i] = footSwingTrajectories[i].getPosition(); //get the desired position
      vFoot_des[i] = footSwingTrajectories[i].getVelocity(); //get the desired velocity
      aFoot_des[i] = footSwingTrajectories[i].getAcceleration(); //get the desired acceleration
    }
    else
    {
      firstSwing[i] = true;
      pFoot_des[i] = pCurContact[i]; //if the foot is in stance, set the desired position to the current position
      vFoot_des[i] = Eigen::Vector3f::Zero(3);
      aFoot_des[i] = Eigen::Vector3f::Zero(3);
    }
  }

  if (_iter % iterationsBetweenMPC == 0) 
  {

    int* mpcTable = walking->getMpcTable();
    for(int i = 0; i < pred_hor; i++)
    {
      for(int j = 0; j < 4; j++)
      {
        _mpcTable(j,i) = mpcTable[i*4 + j];
      }
    }
    std::cout<< "mpcTable: \n" << _mpcTable << std::endl;

    Vec3<T> x  = seResult.position; // + seResult.rBody.transpose()*_model->getComPos();
    std::cout<<"body position :"<< seResult.position.transpose() << std::endl;
    Vec3<T> xd = seResult.vWorld;
    std::cout<<"body velocity: :"<< xd.transpose() << std::endl;
    std::cout<< "com velocity: :" << _model->getComVel().transpose()  << std::endl;
    std::cout<<"contact position \n";
    MatrixXf contact_pos = MatrixXf::Zero(3,4);
    for (int c = 0 ; c < 4; c++){
      contact_pos.col(c) = pCurContact[c];
    }
    std::cout<< contact_pos << std::endl;
    Vec3<T> w  = seResult.omegaBody;
    RotMat<T> R  = seResult.rBody.transpose();
    // std::cout<<"R \n"<< R << std::endl;
    Eigen::Map<Eigen::Matrix<float, 9, 1>> R9by1(R.data());

    State s;
    s.x             = EigenVectorfTodm(x);
    s.xd            = EigenVectorfTodm(xd);
    s.w             = EigenVectorfTodm(w);
    s.R             = EigenVectorfTodm(R9by1);
    s.contactLoc    = DM::zeros(3 * tello_contact::num_foot_contact);
    for (int c = 0; c < tello_contact::num_foot_contact; c++)
    {
      if (contact_state[c] >= 0)
      {
        s.contactLoc(Slice(c*3, c*3+3)) = EigenVectorfTodm(pCurContact[c]);
      }
      else
      {
        s.contactLoc(Slice(c*3, c*3+3)) = EigenVectorfTodm(pNextFoot[c]);
      }
    }
    
    VectorXf cas_x_des = p->CAS_x_des;
    VectorXf cas_v_des = seResult.rBody.transpose() * p->CAS_v_des; //convert to world frame
    VectorXf cas_w_des = p->CAS_w_des;
    Vector3f CAS_RPY_des = p->CAS_RPY_des;
    Eigen::Matrix3f cas_R_des = ori::rpyToRotMat(CAS_RPY_des).transpose();
    // std::cout<<"cas_R_des: \n"<< cas_R_des << std::endl;
    // exit(0);
    //reshape to 9 by 1
    Eigen::Map<Eigen::Matrix<float, 9, 1>> cas_R_des_vec(cas_R_des.data());
    VectorXf ref_traj(18);
    ref_traj << cas_x_des, cas_v_des, cas_w_des, cas_R_des_vec;
    // MatrixXd contact_seq = generate_contact_cycle(p,_iter / iterationsBetweenMPC);
    MPC_result result = opt.optimize(_model, p, s, ref_traj, _mpcTable);
    mpcLogger.send_info(result);
    DM F_RHeel  = result.F_RHeel(Slice(), 0);
    DM F_RToe   = result.F_RToe (Slice(), 0);
    DM F_LHeel  = result.F_LHeel(Slice(), 0);
    DM F_LToe   = result.F_LToe (Slice(), 0);

    std::cout<< dmToEigen(result.F_RHeel) << std::endl;
    std::cout<< dmToEigen(result.F_RToe) << std::endl;
    std::cout<< dmToEigen(result.F_LHeel) << std::endl;
    std::cout<< dmToEigen(result.F_LToe) << std::endl;

    

    DMat<float> mass_matrix = _model->getMassMatrix();
    //order matters
    Fr_des[0] = dmToEigenVectorf(F_RHeel);
    Fr_des[1] = dmToEigenVectorf(F_RToe);
    Fr_des[2] = dmToEigenVectorf(F_LHeel);
    Fr_des[3] = dmToEigenVectorf(F_LToe);

    // std::cout<<"current velocity: \n"<< seResult.vWorld << std::endl;

    // if (cas_v_des[0] >=1.0){
    // Fr_des[0][2] = Fr_des[0][2] * 1.2;
    // Fr_des[1][2] = Fr_des[1][2] * 1.2;
    // Fr_des[2][2] = Fr_des[2][2] * 1.2;
    // Fr_des[3][2] = Fr_des[3][2] * 1.2;
    // }

    // for (int i = 0; i < tello_contact::num_foot_contact; i++)
    // {
    //   std::cout<< i << " Fr_des: \n" << Fr_des[i] << std::endl;
    // }
    pBody_des     = dmToEigenVectorf(result.x(Slice(), 1));
    vBody_des     = dmToEigenVectorf(result.xd(Slice(), 1));
    aBody_des     = Eigen::Vector3f::Zero(3);
    Eigen::Matrix3f R_des = dmToEigenf(reshape(result.xR(Slice(), 1), 3, 3));
    pBody_RPY_des = rotationMatrixToRPY( R_des.transpose() );
    //rotation matrix that rotate around y axis for 30 degree
    // Eigen::Matrix3f temp;
    // temp << cos(30*M_PI/180), 0, sin(30*M_PI/180),
    //         0, 1, 0,
    //         -sin(30*M_PI/180), 0, cos(30*M_PI/180);
    // std::cout<< " temp: \n" << temp << std::endl;
    // std::cout<<"use ori::rpy \n" << rotationMatrixToRPY(temp.transpose()) << std::endl;

    // std::cout<< " pBody_RPY_des: \n" << pBody_RPY_des << std::endl;
    // std::cout<< " pBody_RPY_des eigen method: \n" << R_des.eulerAngles(0,1,2) << std::endl;
    vBody_Ori_des = dmToEigenVectorf(result.xw(Slice(), 1));
    std::cout<< " xd: \n" << dmToEigen(result.xd) << std::endl;
    // exit(0);
    std::cout<< " x: \n" << dmToEigen(result.x) << std::endl;
    // std::cout<< " xR: \n" << result.xR << std::endl;
    // std::cout<< " xw: \n" << result.xw << std::endl;
    // std::cout<< " F_RToe: \n" << result.F_RToe << std::endl;
    // std::cout<< " F_RHeel: \n" << result.F_RHeel << std::endl;
    // std::cout<< " F_LToe: \n" << result.F_LToe << std::endl;
    // std::cout<< " F_LHeel: \n" << result.F_LHeel << std::endl;
  }





  _iter ++;
}

template class CASADI_Walk_Tello<float>;

