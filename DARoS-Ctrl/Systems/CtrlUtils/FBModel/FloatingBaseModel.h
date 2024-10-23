/*! @file FloatingBaseModel.h
 *  @brief Implementation of Rigid Body Floating Base model data structure
 *
 * This class stores the kinematic tree described in "Rigid Body Dynamics
 * Algorithms" by Featherstone (download from
 * https://www.springer.com/us/book/9780387743141 on MIT internet)
 *
 * The tree includes an additional "rotor" body for each body.  This rotor is
 * fixed to the parent body and has a gearing constraint.  This is efficiently
 * included using a technique similar to what is described in Chapter 12 of
 * "Robot and Multibody Dynamics" by Jain.  Note that this implementation is
 * highly specific to the case of a single rotating rotor per rigid body. Rotors
 * have the same joint type as their body, but with an additional gear ratio
 * multiplier applied to the motion subspace. The rotors associated with the
 * floating base don't do anything.
 */

#ifndef LIBBIOMIMETICS_FLOATINGBASEMODEL_H
#define LIBBIOMIMETICS_FLOATINGBASEMODEL_H

#include <string>
#include <vector>
#include <map>

#include <orientation_tools.h>
#include <SpatialInertia.h>
#include <spatial.h>

#include <eigen3/Eigen/StdVector>

using std::vector;
using namespace ori;
using namespace spatial;

/*!
 * The state of a floating base model (base and joints)
 */
template <typename T>
class FBModelState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  FBModelState()
  {
    bodyOrientation.setZero();
    bodyPosition.setZero();
    bodyVelocity.setZero(); // body coordinates
  }
  void operator=(const FBModelState<T> &state)
  {
    this->bodyOrientation = state.bodyOrientation;
    this->bodyPosition = state.bodyPosition;
    this->bodyVelocity = state.bodyVelocity;
    this->q = state.q;
    this->qd = state.qd;
  }

  Quat<T> bodyOrientation;
  Vec3<T> bodyPosition;
  SVec<T> bodyVelocity; // body coordinates
  DVec<T> q;
  DVec<T> qd;

  /*!
     * Print the position of the body
     */
  void print() const
  {
    printf("body position: %.3f %.3f %.3f\n", bodyPosition[0], bodyPosition[1],
           bodyPosition[2]);
    printf("body orientation: %.3f %.3f %.3f %.3f\n", bodyOrientation[0], bodyOrientation[1],
           bodyOrientation[2], bodyOrientation[3]);
    printf("q: \t");
    for (int i(0); i < q.rows(); ++i)
    {
      printf("%.3f, \t", q[i]);
    }
    printf("\n");
  }
};

/*!
 * The result of running the articulated body algorithm on a rigid-body floating
 * base model
 */
template <typename T>
struct FBModelStateDerivative
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3<T> dBodyPosition;
  SVec<T> dBodyVelocity;
  DVec<T> qdd;
};

/*!
 * Class to represent a floating base rigid body model with rotors and ground
 * contacts. No concept of state.
 */
template <typename T>
class FloatingBaseModel
{
public:
  /*!
     * Initialize a floating base model with default gravity
     */
  FloatingBaseModel() : _gravity(0, 0, -9.81) { _bodyNameMap.clear(); }
  ~FloatingBaseModel() {}

  void addBase(const SpatialInertia<T> &inertia, const std::string & link_name = "root");
  void addBase(T mass, const Vec3<T> &com, const Mat3<T> &I, const std::string & link_name = "root");
  int addGroundContactPoint(int bodyID, const Vec3<T> &location,
                            bool isFoot = false);
  void addGroundContactBoxPoints(int bodyId, const Vec3<T> &dims);
  void addGroundContactBoxPointsOffset(int bodyId, const Vec3<T> &dims, const Vec3<T> &offset);

  int addBody(const SpatialInertia<T> &inertia,
              const SpatialInertia<T> &rotorInertia, T gearRatio, int parent,
              JointType jointType, CoordinateAxis jointAxis,
              const Mat6<T> &Xtree, const Mat6<T> &Xrot, const std::string &link_name = "NoName" );
  int addBody(const MassProperties<T> &inertia,
              const MassProperties<T> &rotorInertia, T gearRatio, int parent,
              JointType jointType, CoordinateAxis jointAxis,
              const Mat6<T> &Xtree, const Mat6<T> &Xrot, const std::string &link_name = "NoName");
  void check();
  T totalRotorMass();
  T totalNonRotorMass();

  void addJointLim(size_t jointID, T joint_lim_value_lower, T joint_lim_value_upper);

  /*!
     * Get vector of parents, where parents[i] is the parent body of body i
     * @return Vector of parents
     */
  const std::vector<int> &getParentVector() { return _parents; }

  /*!
     * Get vector of body spatial inertias
     * @return Vector of body spatial inertias
     */
  const std::vector<SpatialInertia<T>,
                    Eigen::aligned_allocator<SpatialInertia<T>>> &
  getBodyInertiaVector()
  {
    return _Ibody;
  }

  /*!
     * Get vector of rotor spatial inertias
     * @return Vector of rotor spatial inertias
     */
  const std::vector<SpatialInertia<T>,
                    Eigen::aligned_allocator<SpatialInertia<T>>> &
  getRotorInertiaVector()
  {
    return _Irot;
  }

  /*!
     * Set the gravity
     */
  void setGravity(Vec3<T> &g) { _gravity = g; }

  /*!
     * Set the flag to enable computing contact info for a given contact point
     * @param gc_index : index of contact point
     * @param flag : enable/disable contact calculation
     */
  void setContactComputeFlag(size_t gc_index, bool flag)
  {
    _compute_contact_info[gc_index] = flag;
  }

  DMat<T> invContactInertia(const int gc_index,
                            const D6Mat<T> &force_directions);
  T invContactInertia(const int gc_index, const Vec3<T> &force_ics_at_contact);

  T applyTestForce(const int gc_index, const Vec3<T> &force_ics_at_contact,
                   FBModelStateDerivative<T> &dstate_out);

  T applyTestForce(const int gc_index, const Vec3<T> &force_ics_at_contact,
                   DVec<T> &dstate_out);

  void addDynamicsVars(int count);

  void resizeSystemMatricies();

  /*!
     * Update the state of the simulator, invalidating previous results
     * @param state : the new state
     */
  void setState(const FBModelState<T> &state)
  {
    _state = state;

    _biasAccelerationsUpToDate = false;
    _compositeInertiasUpToDate = false;
    _compositeBsUpToDate = false;

    resetCalculationFlags();
  }

  /*!
     * Mark all previously calculated values as invalid
     */
  void resetCalculationFlags()
  {
    _articulatedBodiesUpToDate = false;
    _kinematicsUpToDate = false;
    _forcePropagatorsUpToDate = false;
    _qddEffectsUpToDate = false;
    _accelerationsUpToDate = false;
  }

  /*!
     * Update the state derivative of the simulator, invalidating previous results.
     * @param dState : the new state derivative
     */
  void setDState(const FBModelStateDerivative<T> &dState)
  {
    _dState = dState;
    _accelerationsUpToDate = false;
  }

  Vec3<T> getPosition(const int link_idx, const Vec3<T> &local_pos);
  Vec3<T> getPosition(const int link_idx);

  Mat3<T> getOrientation(const int link_idx);
  Vec3<T> getLinearVelocity(const int link_idx, const Vec3<T> &point);
  Vec3<T> getLinearVelocity(const int link_idx);

  Vec3<T> getLinearAcceleration(const int link_idx, const Vec3<T> &point);
  Vec3<T> getLinearAcceleration(const int link_idx);

  Vec3<T> getAngularVelocity(const int link_idx);
  Vec3<T> getAngularAcceleration(const int link_idx);

  void forwardKinematics();
  void biasAccelerations();
  void compositeInertias();
  void compositeBilinearMatrix();
  void forwardAccelerationKinematics();
  void contactJacobians();
  D3Mat<T> getlinkJacobian(size_t link_id, const Vec3<T> pos);
  void FullcontactJacobians();

  DVec<T> generalizedGravityForce();
  DVec<T> generalizedCoriolisForce();
  DMat<T> massMatrix();
  DMat<T> CoriolisMatrixonly();
  void massandCoriolisMatrix();
  DMat<T> centroidMomentumMatrix();
  DVec<T> inverseDynamics(const FBModelStateDerivative<T> &dState);
  void runABA(const DVec<T> &tau, FBModelStateDerivative<T> &dstate);
  void runABAZero();

  size_t _nDof = 0;
  Vec3<T> _gravity;
  vector<int> _parents;
  vector<T> _gearRatios;
  vector<T> _d, _u;

  vector<JointType> _jointTypes;
  vector<CoordinateAxis> _jointAxes;
  vector<Mat6<T>, Eigen::aligned_allocator<Mat6<T>>> _Xtree, _Xrot;
  vector<SpatialInertia<T>, Eigen::aligned_allocator<SpatialInertia<T>>> _Ibody,
      _Irot;

  size_t _nGroundContact = 0;
  int _nJointLim = 0;            //counter for nb of joints limit (is incremented when adding a joint lim to model)
  vector<size_t> _JointLimID;    // hold the joint nb ID as defined in humanoid.h
  vector<T> _JointLimValueLower; // hold the joint nb ID as defined in humanoid.h
  vector<T> _JointLimValueUpper; // hold the joint nb ID as defined in humanoid.h

  vector<size_t> _gcParent;
  vector<Vec3<T>> _gcLocation;
  vector<uint64_t> _footIndicesGC;

  vector<Vec3<T>> _pGC;
  vector<Vec3<T>> _vGC;

  vector<bool> _compute_contact_info;

  /*!
     * Get the mass matrix for the system
     */
  const DMat<T> &getMassMatrix() const { return _H; }

  /*!
     * Get the coriolis matrix for the system
     */
  const DMat<T> &getCorMatrix() const { return _C; }
  /*!
     * Get the gravity term (generalized forces)
     */
  const DVec<T> &getGravityForce() const { return _G; }

  /*!
     * Get the coriolis term (generalized forces)
     */
  const DVec<T> &getCoriolisForce() const { return _Cqd; }

  /*!
     * Get the composite rigid-body inertia of the system
     * about the COM in the centroidal frame
     *
     * Used with CMM, _Ig*_vg = _Ag * qdot
     */
  const Mat6<T> &getCompositeRBI() const { return _Ig; }

  /*!
     * Get the time rate of change (ROC) of the composite
     rigid-body inertia of the system about the COM in the
     centroidal frame
     */
  // const Mat6<T>& getCompositeRBIdot() const { return _Igd; }

  /*!
     * Get the position of the center of mass relative to
     * body frame (expressed in body frame)
     */
  const Vec3<T> &getComPos() const { return _pCOM; }

  /*!
     * Get the position of the center of mass relative to
     * body frame (expressed in world frame)
     */
  const Vec3<T> &getComPosWorld() const { return _pCOM_world; }

  /*!
     * Get the centroidal velocity of the robot expressed
     * in the centroidal frame
     */
  const Vec6<T> &getCentroidalVelocity() const { return _vG; }

  /*!
     * Get the com linear velocity of the robot expressed
     * in the centroidal frame (paralel to world frame)
     */
  const Vec3<T> &getComVel() const { return _vCentroid; }

  /*!
     * Get the centroid momentum matrix
     */
  const DMat<T> &getCMM() const { return _Ag; }

  /*!
     * Get the coriolis term (generalized forces)
     */
  const Vec6<T> &getCmmBiasForce() const { return _Agdqd; }

   const vectorAligned<Mat6<T>> &getLinkXa() const { return _Xa; }
  const vectorAligned<SVec<T>> &getLinkV() const { return _v; }

  size_t getBodyID (const char *body_name) const {
    if (_bodyNameMap.count(body_name) == 0) {
      return std::numeric_limits<unsigned int>::max();
    }
    return _bodyNameMap.find(body_name)->second;
  }
  std::map<std::string, size_t> _bodyNameMap;

  /// BEGIN ALGORITHM SUPPORT VARIABLES
  FBModelState<T> _state;
  FBModelStateDerivative<T> _dState;

  vectorAligned<SVec<T>> _v, _vrot, _a, _arot, _avp, _avprot, _c, _crot, _S,
      _Srot, _fvp, _fvprot, _ag, _agrot, _f, _frot, _dsSrot, _dsS;

  vectorAligned<SVec<T>> _U, _Urot, _Utot, _pA, _pArot;
  vectorAligned<SVec<T>> _externalForces;

  vectorAligned<SpatialInertia<T>> _IC;
  vectorAligned<Mat6<T>> _BC, _Brot;
  vectorAligned<Mat6<T>> _Xup, _Xa, _Xuprot, _IA, _ChiUp; //_Xa contains spatial transforms of each link from world to link frame.

  DMat<T> _H, _C, _Ag;
  DVec<T> _Cqd, _G;
  Vec3<T> _pCOM;
  Vec3<T> _pCOM_world;
  Vec6<T> _vG;
  Vec3<T> _vCentroid;

  Vec6<T> _Agdqd;
  Mat6<T> _Ig;
  //Mat6<T> _Igd;

  vectorAligned<D6Mat<T>> _J;
  vectorAligned<SVec<T>> _Jdqd;

  vectorAligned<D6Mat<T>> _Jc_full;
  vectorAligned<Vec6<T>> _Jcdqd_full;

  vectorAligned<D3Mat<T>> _Jc;
  vectorAligned<Vec3<T>> _Jcdqd;

  bool _kinematicsUpToDate = false;
  bool _biasAccelerationsUpToDate = false;
  bool _accelerationsUpToDate = false;

  bool _compositeInertiasUpToDate = false;
  bool _compositeBsUpToDate = false;

  void updateArticulatedBodies();
  void updateForcePropagators();
  void udpateQddEffects();

  /*!
     * Set all external forces to zero
     */
  void resetExternalForces()
  {
    for (size_t i = 0; i < _nDof; i++)
    {
      _externalForces[i] = SVec<T>::Zero();
    }
  }

  bool _articulatedBodiesUpToDate = false;
  bool _forcePropagatorsUpToDate = false;
  bool _qddEffectsUpToDate = false;

  DMat<T> _qdd_from_base_accel;
  DMat<T> _qdd_from_subqdd;
  Eigen::ColPivHouseholderQR<Mat6<T>> _invIA5;
};

#endif // LIBBIOMIMETICS_FLOATINGBASEMODEL_H
