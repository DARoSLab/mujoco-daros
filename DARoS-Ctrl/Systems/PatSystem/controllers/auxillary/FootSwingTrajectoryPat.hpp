/*!
 * @file FootSwingTrajectoryPat.h
 * @brief Utility to generate foot swing trajectories.
 *
 * Uses Bezier Curves and updates shape of bezier curve based on height map data
 */

#ifndef FootSwingTrajectoryPat_H
#define FootSwingTrajectoryPat_H

#include <systems/patroclus/state_machine/ControlFSMData.h>
#include "cppTypes.h"
#include <cmath>
#include <math.h>
#include <chrono>



//Define swing library options
#define SWING_DEFAULT 0
#define SWING_HIGH 1
#define SWING_BACK 2
#define SWING_FRONT 3
#define SWING_IN 4
#define SWING_OUT 5

/*!
 * A foot swing trajectory for a single foot
 */
template<typename T>
class FootSwingTrajectoryPat {
public:

  /*!
   * Construct a new foot swing trajectory with everything set to zero
   */
  FootSwingTrajectoryPat() {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _height = 0;

    rot_z.setZero();
    offset.setZero();
    rot_z(2,2)=1;
    max_height=.2;
    max_x_offset=.2;
    library_size=6;
    min_x_offset=.0;
    height_tolerance=.02; //[cm] allowed
    bezier_height_factor=4.0/3.0; //linear factor between effective height and bezier offset
    traj_type=SWING_DEFAULT;
    offset<<0.0f,0.0f,0.0f;
    // _q<<0.0f,0.0f,0.0f;
    //set swing library
    offsetLibrary[SWING_DEFAULT]<<0.0f,  0.0f,  0.0f;
    // offsetLibrary[SWING_DEFAULT][0]=0;
    // offsetLibrary[SWING_DEFAULT][1]=0;
    // offsetLibrary[SWING_DEFAULT][2]=0;

    offsetLibrary[SWING_HIGH]<<0.0f,  0.0f,  .2f;

    offsetLibrary[SWING_BACK]<<-0.5f,  0.0f,  0.0f;

    offsetLibrary[SWING_FRONT]<<0.5f,  0,  0;

    offsetLibrary[SWING_IN]<<0,  -.06,  0;

    offsetLibrary[SWING_OUT]<<0,  .08,  0;
  }

  /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position
   */
  void setInitialPosition(Vec3<T> p0) {
    _p0 = p0;
  }

  /*!
   * Get the starting location of the foot
   * @return : the initial foot position
   */
  Vec3<T> getInitialPosition() {
    return _p0;
  }

  /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton
   */
  void setFinalPosition(Vec3<T> pf) {
    _pf = pf;
  }

  /*!
   * Get the desired final position of the foot
   * @return : the final foot posiiton
   */
  Vec3<T> getFinalPosition() {
    return _pf;
  }

  /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the swing
   */
  void setHeight(T h) {
    _height = h;
  }

 /*!
   * Set the index for the trajectory type
   * @param typ: 
   */
  void setTrajectoryType(int typ) {
    traj_type = typ;
  } 

  /*!
   * Get the index for the trajectory type
   * @param typ: 
   */
  int getTrajectoryType() {
    return traj_type;
  } 

   /*!
   * Set the XYZ offset of the swing in the swing frame
   * @param h : 
   */
  void setOffset(T x_off, T y_off, T z_off) {
    offset[0]=diff[0]*x_off;
    offset[1]=diff[1]*y_off;
    offset[2]=diff[2]*z_off+_height;
  }

  /*!
   * Set the Bezier offsets and control points for the specified trajectory type
   * @param h : 
   */
  void setBezierTrajectory(){
    diff=_pf-_p0;
    // std::cout<<"X: "<<offsetLibrary[traj_type][0]<<"\nY: "<<offsetLibrary[traj_type][1]<<"\nZ: "<<offsetLibrary[traj_type][2]<<"\n";
    offset[2]=(_height+ offsetLibrary[traj_type][2])*bezier_height_factor;
    if (getTrajectoryType() == SWING_DEFAULT || getTrajectoryType() == SWING_HIGH){ //default trajectory
      offset[0]=0;
      offset[1]=0;
    }
    else{ 
      xy_dist=sqrt(diff[0]*diff[0]+diff[1]*diff[1]); //Could speed up by only doing this if traj has xy offsets, could no norm as well
      offset[0]=offsetLibrary[traj_type][0]>0 ? offsetLibrary[traj_type][0]*xy_dist+min_x_offset : offsetLibrary[traj_type][0]*xy_dist-min_x_offset ;
      offset[1]=offsetLibrary[traj_type][1]*_side_sign*bezier_height_factor; 

      if (offset[0]> max_x_offset){ //bound x_offset
        offset[0]= max_x_offset;
      }  else if(offset[0]< -max_x_offset){
        offset[0]= -max_x_offset;
      }
   
    }
  

    
    if (offset[2]> max_height){ //bound height
      offset[2]= max_height;
    }
    
    setBezierControlPoints();
    //can be sped up by not calculating unnecessary stuff for the default trajectory
  }

  /*!
   * Set the Bezier control points for the position, velocity and acceleration
   */
  void setBezierControlPoints(){
    // diff=_pf-_p0;
    theta=atan2(diff[1],diff[0]);
    c1=cos(theta);
    s1=sin(theta);
    rot_z(0,0)=c1;
    rot_z(0,1)=-s1;
    rot_z(1,0)=s1;
    rot_z(1,1)=c1;
    //position
    wp[0]=_p0;
    wp[1]=_p0+rot_z*offset; // maybe print this
    wp[2]=_pf+rot_z*offset;
    wp[3]=_pf;
    //velocity
    wv[0]=3*(wp[1]-wp[0]);
    wv[1]=3*(wp[2]-wp[1]);
    wv[2]=3*(wp[3]-wp[2]);
    //acceleration
    wa[0]=2*(wv[1]-wv[0]);
    wa[1]=2*(wv[2]-wv[1]);
  }

  Vec3<T> getBezierControlPoint1(){
    return wp[1];
  }
  Vec3<T> getBezierControlPoint2(){
    return wp[2];
  }

  


  void updateTrajectoryBehavior(ControlFSMData<T>& data,
     const DMat<T> & height_map, const DMat<int> & idx_map, int traj_collision_pts,int shin_collision_pts, int leg, T swingState, T timeout, Vec3<T> & footstep_ptr);

  void updateTouchdown(ControlFSMData<T>& data,
     const DMat<T> & height_map, const DMat<int> & idx_map, int traj_collision_pts,int shin_collision_pts, int leg, T swingState, T timeout, Vec3<T> & footstep_ptr);

  void computeSwingTrajectoryBezier(T phase, T swingTime);

  void computeSwingTrajectoryBezierNew(T phase, T swingTime, T side_sign);

  int predictTrajCollision(ControlFSMData<T>& data,
      const DMat<T> & height_map,int traj_collision_pts,int shin_collision_pts, int leg, T swingState);

  int legCollisionCheck(ControlFSMData<T>& data,
     const DMat<T> & height_map,int traj_collision_pts,int shin_collision_pts, int leg, T phase, Vec3<T> q0);

  void exploreTouchdown(ControlFSMData<T>& data,
     const DMat<T> & height_map, const DMat<int> & idx_map,int traj_collision_pts,int shin_collision_pts, int leg, T phase);

  bool pointCollisionCheck(Vec3<T> coord,const DMat<T> & height_map);  
  bool pointTravCheck(Vec3<T> coord,const DMat<int> & idx_map);

  void computeBezier(T t, T swingTime);

  void moveFootstep(Vec3<T> & footstep,  T dist);

  //easier to implement in FSM

  void initialize(int leg, int side_sign, Vec3<T> p0, T height){
    /*!
    * Initialize all the variables at the beginning of a controller. To run only once
    */
    _leg=leg;
    _side_sign=side_sign;
    setTrajectory(p0,p0,height);
  }

  void visualize(ControlFSMData<T>& data, T swingState);

  void setTrajectory(Vec3<T> initialPos, Vec3<T> finalPos, T height_des){
    _p0=initialPos;
    _pf=finalPos;
    _height=height_des;
    setBezierTrajectory(); //could combine w initial and final
  }


  /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
  Vec3<T> getPosition() {
    return _p;
  }

  /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
  Vec3<T> getVelocity() {
    return _v;
  }

  /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
  Vec3<T> getAcceleration() {
    return _a;
  }

  /*!
   * Set the initial abad position 
   * @param abad_pos :
   */
  void setInitialAbadPosition(Vec3<T> abad_pos) {
    initial_abad_pos=abad_pos;
  }

  /*!
   * Get the initial abad position during takeoff (phase=0)
   * @return : the abad position
   */
  Vec3<T> getInitialAbadPosition() {
    return initial_abad_pos;
  }

  // /*!
  //  * Set the initial abad position 
  //  * @param abad_pos :
  //  */
  // void setJoints(Vec3<T> q0) {
  //   _q=q0;
  // }

  // /*!
  //  * Get the initial abad position during takeoff (phase=0)
  //  * @return : the abad position
  //  */
  // Vec3<T> getJoints() {
  //   return _q;
  // }


private:
  // Vec3<T> _joints;
  Vec3<T> _p0, _pf, _p, _v, _a, initial_abad_pos, diff, offset, wp[4], wv[3],wa[2];
  Vec3<T> _body_pos_ground, _abad_pos, _foot_pos, _q0;
  T _height, max_height, min_x_offset, max_x_offset, height_tolerance; //min_x_offset
  T theta, c1, s1,xy_dist;
  T bezier_height_factor;
  // T t2,t3,mt,mt2,mt3,f0,f1,f2,f3,basis; //initialize computeBezier variables
  // int library_idx;
  int  _leg, traj_type, library_size, _side_sign;
  Vec3<T> offsetLibrary[10];
  Mat3<T> rot_z;
  double grid_size=.01;

};


#endif //CHEETAH_SOFTWARE_FootSwingTrajectoryPat_HPP
