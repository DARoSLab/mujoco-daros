#ifdef RMPFLOW_BUILD
#ifndef FSM_State_RMP_H
#define FSM_State_RMP_H

#include <FSM_State.h>
#include <common/control/RMPFlow/RMPNode.hpp>
#include <Goldfarb_Optimizer/QuadProg++.hh>


/**
 *
 */
template <typename T>
class FSM_State_RMP : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_RMP(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int _iter = 0;
  T _phase[2];
  T _gait_period = 6*0.33;

  // Parses contact specific controls to the leg controller
  void BalanceStandStep();
  void UpdateGaitInfo();
  T SwingTrajectory(T phase, T swing_height);
  RMPLeaf<float>* _rf_attractorRMP;
  RMPLeaf<float>* _lf_attractorRMP;
  RMPLeaf<float>* _rf_redundancyRMP;
  RMPLeaf<float>* _lf_redundancyRMP;
  RMPLeaf<float>* _lf_collisionRMP_1;
  RMPIntermediate<float>* _lf_FKRMP;
  RMPIntermediate<float>* _rf_FKRMP;
  RMPRoot<float>* _rootRMP;
  FloatingBaseModel<float> _model;
  FBModelState<T> _state;
  DVec<T> _qdot_des, _q_des;
  DMat<T> _A;
  DMat<T> _Ainv;
  DVec<T> _grav;
  DVec<T> _coriolis;
  DVec<T> _tau_ff;
  DVec<T> _qddot;
  DVec<T> _collision_1;

  RMPLeaf<float>* _lf_c[4];
  RMPLeaf<float>* _left_shank_c[4][4];
  Vec3<T> _collision_loc[2][4];
  Vec3<T> _shank_offsets[2][4];

  RMPIntermediate<float>* _foot_FKRMP[2];
  RMPIntermediate<float>* _shank_FKRMP;
  RMPLeaf<float>* _foot_attractorRMP[2];
  RMPLeaf<float>* _shank_collisionRMP;
  Vec3<T> _zero_3;
  Vec3<T> _witness_pts[2];
  bool find_shank_capsule_witness_pts(Vec3<T>&wp1, Vec3<T>&wp2);
  bool compute_witness_pts(Vec3<T>p1, Vec3<T>p2, Vec3<T>p3, Vec3<T>p4, T cp1_r, T cp2_r,

                                            Vec3<T>&wp1, Vec3<T>&wp2);

void eig_to_gold_mat(DMat<T> eig, GolDIdnani::GMatr<double> &gold, size_t row, size_t col);
void eig_to_gold_vec(DVec<T> eig, GolDIdnani::GVect<double> &gold, size_t size);
};

#endif  // FSM_State_RMP_H
#endif  // FSM_State_RMP_H
