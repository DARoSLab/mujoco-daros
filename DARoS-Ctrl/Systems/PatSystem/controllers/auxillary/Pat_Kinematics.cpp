
#include <cmath>
#include <iostream>
#include <type_traits>

#include "cppTypes.h"
#include "Pat_Kinematics.h"

#include <utils/Utilities/Mathutilities.h>
#include <math.h>
#include <utils/Utilities/pseudoInverse.h>

template <typename T>
Pat_Kinematics<T>::Pat_Kinematics(PatBiped<T>* pat, T max_iter, T tolerance)
{
    _max_iter  = max_iter;
    _tolerance = tolerance;
    _model = pat->buildModel();

    _state.bodyVelocity.setZero();
    _state.bodyPosition.setZero();
    _state.bodyOrientation.setZero();
    _state.bodyOrientation(0) = 1.; // for zero rpy, quaternion is (1,0,0,0)

    _state.q = DVec<T>::Zero(_model._nDof-6);
    _state.qd = DVec<T>::Zero(_model._nDof-6);

    std::cout << "[Pat_Kinematics] Initialized : " << std::endl;
  }



  /*!
   * Finds joint angles for a desired position p of a point located at rel_pos
   * in link frame .
   * Returns joint angles q
   */
  template <typename T>
  DVec<T> Pat_Kinematics<T>::IK(Vec3<T>p, DVec<T> q0,int linkid, Vec3<T> rel_pos){
    clock_t t;
    t = clock();//start timer
    DVec<T> q, dq;
    Vec3<T> e;
    DMat<T> J,J_ps;

    int Nbjoint = getNumJoints(linkid); // counts number of joints from link to body
    // std::cout<< Nbjoint << "jointcounts" <<std::endl;
    int iter = 0;

    //Find qd st f(q)=X-f(qd)=0
    //Uses Newton-Raphson method

    double sigma_thresh=.01;

    //Initial guess
    if (q0.size() != Nbjoint){
      throw std::invalid_argument("Inverse kinamatics :initial guess has wrong number of joint angles");//raise error
    }

    q = q0;
    UpdateModel(q,linkid, Nbjoint);// updated the model with initial guess angles
    e = p-FK(linkid,rel_pos); //error vector

    // if (e.norm()>quad._abadLinkLength+quad._hipLinkLength+quad._kneeLinkLength){//position is out of bounds
    //   throw std::invalid_argument("Relative position is out of bounds");//raise error
    //   std::cout<<"Leg: "<<leg<<"  Error: "<<e.norm()<<"\n";
    // }

    while (e.norm()>_tolerance && iter < _max_iter){ //while the error distance is smaller than some tolerance
      J = Jlink(linkid, Nbjoint, rel_pos);

      pseudoInverse(J,sigma_thresh,J_ps); //Compute J_ps

      dq=J_ps*e;

      q+= dq; //calculate new q (Matrix Vector Multiplication)

      UpdateModel(q,linkid, Nbjoint);// updated the model updated angles

      e = p - FK(linkid,rel_pos); //error vector

      iter++;
    }

    //Print useful info
    // std::cout<<"Link: "<<linkid<<"  Iter: "<<iter<<" dq: "<<dq<<"  Error: "<<e.norm()<<"\n";

    t = clock() - t;
    // printf ("IK took %f seconds.\n",((float)t)/CLOCKS_PER_SEC);
    return q;

  }


  /*!
   * Finds joint angles for a desired position p of a point located at rel_pos
   * in link frame, based on a JTranspose method (no inverse/peusoinverse required)
   * Returns joint angles (SLOWER ~1ms)
   */
  template <typename T>
  DVec<T> Pat_Kinematics<T>::IK_Jt(Vec3<T>p, DVec<T> q0,int linkid, Vec3<T> rel_pos){
    clock_t t;
    t = clock();//start timer
    DVec<T> q, dq;
    Vec3<T> e;
    DMat<T> J;
    T alpha;


    int Nbjoint = getNumJoints(linkid); // counts number of joints from link to body
    // std::cout<< Nbjoint << "jointcounts" <<std::endl;
    int iter = 0;



    //Initial guess
    if (q0.size() != Nbjoint){
      throw std::invalid_argument("Inverse kinamatics :initial guess has wrong number of joint angles");//raise error
    }

    q = q0;
    UpdateModel(q,linkid, Nbjoint);// updated the model with initial guess angles
    e = p-FK(linkid,rel_pos); //error vector

    // if (e.norm()>quad._abadLinkLength+quad._hipLinkLength+quad._kneeLinkLength){//position is out of bounds
    //   throw std::invalid_argument("Relative position is out of bounds");//raise error
    //   std::cout<<"Leg: "<<leg<<"  Error: "<<e.norm()<<"\n";
    // }

    while (e.norm()>_tolerance && iter < _max_iter){ //while the error distance is smaller than some tolerance
      J = Jlink(linkid, Nbjoint, rel_pos);

      Vec3<T> t1 = J*J.transpose()*e;
      T num = e.transpose()*t1;
      T denom = t1.transpose()*t1;
      alpha = num/denom;

      dq = alpha*J.transpose()*e;


      q+= dq; //calculate new q (Matrix Vector Multiplication)

      UpdateModel(q,linkid, Nbjoint);// updated the model updated angles

      e = p - FK(linkid,rel_pos); //error vector

      //Print useful info
      // std::cout<<"Link: "<<linkid<<"  Iter: "<<iter<<" dq: "<<dq<<"  Error: "<<e.norm()<<"\n";
      iter++;
    }
    t = clock() - t;
    // printf ("It took me %f seconds.\n",((float)t)/CLOCKS_PER_SEC);
    // std::cout<<"Link: "<<linkid<<"  Iter: "<<iter<<" dq: "<<dq<<"  Error: "<<e.norm()<<"\n";


    return q;

  }

template<typename T>
D3Mat<T> Pat_Kinematics<T>::Jlink(int linkid,int Nbjoint, Vec3<T> rel_pos){
  D3Mat<T> A = _model.getlinkJacobian(linkid,rel_pos); // right ankle jacobian;
  D3Mat<T> J = D3Mat<T>::Zero(3,Nbjoint);

  for (int i(0);i<Nbjoint;i++){
    J.col(i) = A.col(i+linkid - Nbjoint + 1);//(i + 6)
  }
  return J;
}
template<typename T>
void Pat_Kinematics<T>::getLegJacobian(int leg, DMat<T>* J){//world coordinate
  _model.contactJacobians();
  auto Jc  = _model._Jc[10+3*leg];
  for(int i=0; i<3; i++){
    for(int j=0; j<3;j++){
      J->operator()(i, j) = Jc(i, 6 + 3*leg + j);
    }
  }
}

template<typename T>
Vec3<T> Pat_Kinematics<T>::FK(int linkid, Vec3<T> rel_pos){
   _model.forwardKinematics();
  Vec3<T> pos = _model._pGC[linkid]; // get position
  return pos;
}
template<typename T>
void Pat_Kinematics<T>::FK(int leg, Vec3<T>* pos){
  size_t gcID = _model._footIndicesGC.at(leg);
  pos->operator()(0)= _model._pGC[gcID](0);
  pos->operator()(1)= _model._pGC[gcID](1);
  pos->operator()(2)= _model._pGC[gcID](2);
}
template<typename T>
void Pat_Kinematics<T>::getFootVelocity(int leg, Vec3<T>* vel){
  /*Foot velocity in hip frame */
  size_t gcID = _model._footIndicesGC.at(leg);
  vel->operator()(0)= _model._vGC[gcID](0);
  vel->operator()(1)= _model._vGC[gcID](1);
  vel->operator()(2)= _model._vGC[gcID](2);
}
template<typename T>
void Pat_Kinematics<T>::computeFootPosVelJacobian(int leg, Vec3<T>* pos, Vec3<T>* vel, DMat<T>* J){
  /*Foot velocity in hip frame */
  FK(leg, pos);
  getLegJacobian(leg, J);
  getFootVelocity(leg, vel);
}
template<typename T>
void Pat_Kinematics<T>::UpdateModel(const can_data_lcmt* canData){

  for (int leg(0);leg < 2;++leg){
    _state.q[3*leg + 0] = canData->q_abad[leg];
    _state.q[3*leg + 1] = canData->q_hip[leg];
    _state.q[3*leg + 2] = canData->q_knee[leg];
    _state.qd[3*leg + 0] = canData->qd_abad[leg];
    _state.qd[3*leg + 1] = canData->qd_hip[leg];
    _state.qd[3*leg + 2] = canData->qd_knee[leg];
  }
  _model.setState(_state);
  // _model.forwardKinematics();

}

template<typename T>
void Pat_Kinematics<T>::UpdateModel(DVec<T> joint_angles,int linkid, int Nbjoint){
  // TODO Charles generalize this for any joints, any nb of jointss

  for (int i(0);i < Nbjoint;++i){
    _state.q[i + linkid - Nbjoint + 1 - 6] = joint_angles[i];
  }


  _model.setState(_state);

}
template<typename T>
void Pat_Kinematics<T>::UpdateModel(int leg, DVec<T> &q, DVec<T> &qd){
  for (int i(0);i < 3;++i){
    _state.q[3*leg + i] = q(i);
    _state.qd[3*leg + i] = qd(i);
  }
  _model.setState(_state);


}



/*!
 * Returns number of joints from link to body
 */
template<typename T>
int Pat_Kinematics<T>::getNumJoints(int linkid){
  int i = linkid;
  int jcount = 0;

  while (i > 5) {
    i = _model._parents[i];
    jcount++;
  }
  return jcount;
}

template class Pat_Kinematics<float>;
template class Pat_Kinematics<double>;
