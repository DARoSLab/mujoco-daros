/*! @file LegControllerPat.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards (the low level leg control boards)
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#ifndef PATROCLUS_LEGCONTROLLER_H
#define PATROCLUS_LEGCONTROLLER_H

#include "cppTypes.h"
#include "pat_leg_control_command_lcmt.hpp"
#include "pat_leg_control_data_lcmt.hpp"
#include <robots/PatBiped.h>
#include <pretty_print.h>
#include <can_data_lcmt.hpp>
#include <can_command_lcmt.hpp>
#include <ctrl_utils/LimbData.hpp>
#include "Pat_Kinematics.h"
//#include "SimUtilities/SpineBoard.h"
//#include "SimUtilities/ti_boardcontrol.h"

/*!
 * Data sent from the control algorithm to the legs.
 */
template <typename T>
struct LegControllerPatCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerPatCommand() { zero(); }

  void zero();

  void print(){
    pretty_print(qDes, std::cout, "q Des");
    pretty_print(qdDes, std::cout, "qd Des");
    pretty_print(kpJoint, std::cout, "Kp Joint");
    pretty_print(kdJoint, std::cout, "Kd Joint");
  }

  void print_cartesian(){
    pretty_print(pDes, std::cout, "p Des");
    pretty_print(vDes, std::cout, "v Des");
    pretty_print(kpCartesian, std::cout, "Kp Cartesian");
    pretty_print(kdCartesian, std::cout, "Kd Cartesian");
  }


  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
  Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

/*!
 * Data returned from the legs to the control code.
 */
template <typename T>
struct LegControllerPatData: public LimbData<T> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerPatData():LimbData<T>(){
    this->J = DMat<T>::Zero(3,3);
    this->q = DVec<T>::Zero(3);
    this->qd = DVec<T>::Zero(3);
    this->tauEstimate = DVec<T>::Zero(3);
    this->tauAct = DVec<T>::Zero(3);
    zero();
  }
  virtual ~LegControllerPatData(){}

  //void setPat(PatBiped<T>& pat) { patruped = &pat; }

  virtual void zero();
  void print(){
    pretty_print(this->q, std::cout, "q");
    pretty_print(this->qd, std::cout, "qd");
  }

  //Vec3<T> q, qd;
  //Mat3<T> J;
  //Vec3<T> tauEstimate;
  //PatBiped<T>* patruped;
};

/*!
 * Controller for pat_biped::num_legs legs of a patruped.  Works for both Mini Cheetah and Cheetah 3
 */
template <typename T>
class LegControllerPat {
 public:
  LegControllerPat(PatBiped<T>& pat) : _pat(pat) {
    //for (auto& data : datas) data.setPat(pat);
    datas = new LimbData<T>* [pat_biped::num_legs];
    for(size_t i(0); i<pat_biped::num_legs; ++i) datas[i] = new LegControllerPatData<T>();
    _FK = new Pat_Kinematics<T>(&_pat, 6, 0.0005); // last two arguments define max iterations and tolerance for IK

  }
  ~LegControllerPat(){
    for(size_t i(0); i<pat_biped::num_legs; ++i) delete datas[i];
    delete [] datas;
  }

  void zeroCommand();
  void edampCommand(RobotType robot, T gain);
  void updateData(const can_data_lcmt* canData);
  void updateCommand(can_command_lcmt* canCommand);
  void setEnabled(bool enabled) { _legsEnabled = enabled; };
  void setLcm(pat_leg_control_data_lcmt* data, pat_leg_control_command_lcmt* command);
  void computeFKJacobian(int leg, DVec<T>& q, Vec3<T>* pos, DMat<T>* J);
  void updateModelOriAndVel(Vec4<T> q, Vec3<T> omegaBody);
  /*!
   * Set the maximum torque.  This only works on cheetah 3!
   */
  void setMaxTorqueCheetah3(T tau) { _maxTorque = tau; }

  LegControllerPatCommand<T> commands[pat_biped::num_legs];
  LimbData<T>** datas;
  PatBiped<T>& _pat;
  Pat_Kinematics<T>* _FK;
  bool _legsEnabled = false;
  T _maxTorque = 0;
  bool _zeroEncoders = false;
  u32 _calibrateEncoders = 0;

  T _V = _pat._batteryV;
  T _gr[3] = {_pat._abadGearRatio, _pat._hipGearRatio, _pat._kneeGearRatio};
  T _kt[3] = {_pat._motorKT, _pat._motorKT, _pat._motorKT};
  T _R[3] = {_pat._motorR, _pat._motorR, _pat._motorR};
  T _tauMax[3] = {_pat._motorTauMax, _pat._motorTauMax, _pat._motorTauMax};

};

template <typename T>
void computeLegJacobianAndPosition(PatBiped<T>& pat, DVec<T>& q, DMat<T>* J,
                                   Vec3<T>* p, int leg);

#endif  // PROJECT_LEGCONTROLLER_H
