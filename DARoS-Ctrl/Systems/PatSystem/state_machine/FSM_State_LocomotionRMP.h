#ifdef RMPFLOW_BUILD
#ifndef PAT_RMP_LOCOMOTION_H
#define PAT_RMP_LOCOMOTION_H

#include <convexMPC_Biped/cMPC_BipedLocomotion.h>
#include <convexMPC_Biped/WBC_BipedLocomotion.h>
#include <common/control/RMPFlow/RMPNode.hpp>
#include <common/control/RMPFlow/LeafNodeSet/LinkCollisionRMP.hpp>
#include <Goldfarb_Optimizer/QuadProg++.hh>
#include <FSM_State.h>
#include <lcm-cpp.hpp>
#include "rmp_lcmt.hpp"

// #define USE_CMPC
template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlRMPData;
/**
 *
 */
template <typename T>
class FSM_State_LocomotionRMP : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_LocomotionRMP(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int iter = 0;
  #ifdef USE_CMPC
    cMPC_BipedLocomotion* cMPC_biped;
  #else
    WBC_BipedLocomotion* cMPC_biped;
  #endif
  bool ESTOP = false;
  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlRMPData<T> * _wbc_data;
  const FloatingBaseModel<T> * _model;
  FloatingBaseModel<T>  _rmp_model;
  FBModelState<T> _rmp_state;
  // FloatingBaseModel<T> _model;
  Vec3<T> _ini_body_ori_rpy;

  Vec3<T> rpy_ini;
  Vec4<T> _ori_ini_inv;

  bool first_visit = true;
  // Parses contact specific controls to the leg controller

  Vec3<T> _zero_3;
  RMPIntermediate<float>* _foot_FKRMP[pat_biped::num_legs];
  RMPLeaf<float>* _foot_attractorRMP[pat_biped::num_legs];
  RMPIntermediate<float>* _shank_FKRMP;
  RMPRoot<float>* _rootRMP;

  // Vec3<T> _collision_loc[2][4];
  Vec3<T> _shank_offsets[2][4];
  Vec3<T> _collision_loc;
  Vec3<T> _control_loc;

  LinkCollisionRMP<float>* _shank_collisionRMP;
  Vec3<T> _witness_pts[2];

  lcm::LCM _rmpLCM;
  rmp_lcmt _rmp_lcm;

  T _min_distance = 1e8;


  void updateRMPState();
  void dump_exit_status();
  bool find_shank_capsule_witness_pts(Vec3<T>&wp1, Vec3<T>&wp2);
  bool compute_witness_pts(Vec3<T>p1, Vec3<T>p2, Vec3<T>p3, Vec3<T>p4, T cp1_r, T cp2_r,
                                            Vec3<T>&wp1, Vec3<T>&wp2);

  void eig_to_gold_mat(DMat<T> eig, GolDIdnani::GMatr<double> &gold, size_t row, size_t col);
  void eig_to_gold_vec(DVec<T> eig, GolDIdnani::GVect<double> &gold, size_t size);
  void LocomotionControlStep();

};

#endif  // FSM_STATE_LOCOMOTION_H
#endif
