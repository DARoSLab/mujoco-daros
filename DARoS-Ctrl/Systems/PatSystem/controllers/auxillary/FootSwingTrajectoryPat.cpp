/*!
 * @file FootSwingTrajectoryPat.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Uses Bezier Curves and updates shape of bezier curve based on height map data
 */

#include <utils/Utilities/Interpolation.h>
#include "PatKinematics.h"
#include "FootSwingTrajectoryPat.hpp"
// #include <Utilities/Utilities_print.h>

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */

template <typename T>
void FootSwingTrajectoryPat<T>::computeSwingTrajectoryBezierNew(T phase, T swingTime, T side_sign) {
  _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
  // printf("Height  %f \n",this->_data->_stateEstimator->getResult().rBody(2,2));
  // printf("Time 2 %f \n", phase[1]);
  // printf("Time 3 %f \n", phase[2]);
  // printf("Time 4 %f \n", phase[3]);
  // printf("Time 5 %f \n", phase[4]);
  // printf("Time 6 %f \n", phase[5]);
  _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase) / swingTime;
  _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);

  T zp, zv, za, yp, yv, ya;
  float y_height=.05;
  if (side_sign>0) y_height=y_height;
  else y_height=-y_height;

  if(phase < T(0.5)) {
    zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _height, phase * 2);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
  } else {
    zp = Interpolate::cubicBezier<T>(_p0[2] + _height, _pf[2], phase * 2 - 1);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
  }
  //   if(phase < T(0.5)) {
  //   yp = Interpolate::cubicBezier<T>(_p0[1], _p0[1] + y_height, phase * 2);
  //   yv = Interpolate::cubicBezierFirstDerivative<T>(_p0[1], _p0[1] + y_height, phase * 2) * 2 / swingTime;
  //   ya = Interpolate::cubicBezierSecondDerivative<T>(_p0[1], _p0[1] + y_height, phase * 2) * 4 / (swingTime * swingTime);
  // } else {
  //   yp = Interpolate::cubicBezier<T>(_p0[1] + y_height, _pf[2], phase * 2 - 1);
  //   yv = Interpolate::cubicBezierFirstDerivative<T>(_p0[1] + y_height, _pf[1], phase * 2 - 1) * 2 / swingTime;
  //   ya = Interpolate::cubicBezierSecondDerivative<T>(_p0[1] + y_height, _pf[1], phase * 2 - 1) * 4 / (swingTime * swingTime);
  // }
  // printf("New height  %f \n", _p[2]);

  _p[2] = zp;
  _v[2] = zv;
  _a[2] = za;


  // _p[1] = yp;
  // _v[1] = yv;
  // _a[1] = ya;
}

template <typename T>
void FootSwingTrajectoryPat<T>::computeSwingTrajectoryBezier(T phase, T swingTime) {
  _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
  _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase) / swingTime;
  _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);

  T zp, zv, za;

  if(phase < T(0.5)) {
    zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _height, phase * 2);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
  } else {
    zp = Interpolate::cubicBezier<T>(_p0[2] + _height, _pf[2], phase * 2 - 1);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
  }

  _p[2] = zp;
  _v[2] = zv;
  _a[2] = za;
}

template <typename T>
void FootSwingTrajectoryPat<T>::computeBezier(T t, T swingTime){
    T t2,t3,mt,mt2,mt3,f0,f1,f2,f3,basis, swingTime2;
    t2=t*t;
    t3=t2*t;
    mt=1-t;
    mt2=mt*mt;
    mt3=mt2*mt;
    swingTime2=swingTime*swingTime;
    //position
    f0=mt3;
    f1=3*mt2*t;
    f2=3*mt*t2;
    f3=t3;
    basis=f0+f1+f2+f3; //constant
    _p[0]=(f0*wp[0][0]+f1*wp[1][0]+f2*wp[2][0]+f3*wp[3][0])/basis;
    _p[1]=(f0*wp[0][1]+f1*wp[1][1]+f2*wp[2][1]+f3*wp[3][1])/basis;
    _p[2]=(f0*wp[0][2]+f1*wp[1][2]+f2*wp[2][2]+f3*wp[3][2])/basis;
    //velocity
    f0=mt2;
    f1=2*mt*t;
    f2=t2;
    basis=f0+f1+f2;
    _v[0]=((f0*wv[0][0]+f1*wv[1][0]+f2*wv[2][0])/basis)/swingTime;
    _v[1]=((f0*wv[0][1]+f1*wv[1][1]+f2*wv[2][1])/basis)/swingTime;
    _v[2]=((f0*wv[0][2]+f1*wv[1][2]+f2*wv[2][2])/basis)/swingTime;
    //acceleration
    f0=mt;
    f1=t;
    basis=f0+f1;
    _a[0]=((f0*wa[0][0]+f1*wa[1][0])/basis)/(swingTime2);
    _a[1]=((f0*wa[0][1]+f1*wa[1][1])/basis)/(swingTime2);
    _a[2]=((f0*wa[0][2]+f1*wa[1][2])/basis)/(swingTime2);
  }

template<typename T> 
bool FootSwingTrajectoryPat<T>::pointCollisionCheck(Vec3<T> coord,const DMat<T> & height_map){
    int row_idx_half = height_map.rows()/2;
    int col_idx_half = height_map.cols()/2;

    int x_idx = -floor(coord[0]/grid_size) + row_idx_half;
    int y_idx = -floor(coord[1]/grid_size) + col_idx_half;

    int x_idx_selected = x_idx;
    int y_idx_selected = y_idx;

    //_IdxMapChecking(x_idx, y_idx, x_idx_selected, y_idx_selected, idx_map);
    //if (x_idx != x_idx_selected || y_idx != y_idx_selected) {
      //foot[0] = (x_idx_selected - row_idx_half)*grid_size + body_pos[0];
      //foot[1] = (y_idx_selected - col_idx_half)*grid_size + body_pos[1];
      //foot[2] = height_map(x_idx_selected, y_idx_selected);
    //}

    // foot[0] = -(x_idx_selected - row_idx_half)*grid_size + body_pos[0];
    // foot[1] = -(y_idx_selected - col_idx_half)*grid_size + body_pos[1];
    if(x_idx_selected < 0 || x_idx_selected > height_map.rows()){
      printf("[WARNING] selected x index is outside of the height map: %d\n", x_idx_selected );
      x_idx_selected = x_idx;
    }
    if(y_idx_selected < 0 || y_idx_selected > height_map.cols()){
      printf("[WARNING] selected y index is outside of the height map: %d\n", y_idx_selected );
      y_idx_selected = y_idx;
    }
    //  foot[2] = height_map(x_idx_selected, y_idx_selected);
    // std::cout << "x_coord: " << coord[0] << "\n";
    // std::cout << "y_coord: " << coord[1] << "\n";
    // std::cout << "x_idx: " << x_idx << "\n";
    // std::cout << "y_idx: " << y_idx << "\n";
    // std::cout << "columns: " << height_map.cols() << "\n";
    // std::cout << "rows: " << height_map.rows() << "\n";
    float height=height_map(x_idx_selected, y_idx_selected);
    
    // if (height>0.01){
    //     std::cout<<"POINT HAS NONZERO HEIGHT "<<height;
    // }
    // std::cout << "Ground Height: " << height << "\n";
    // std::cout << "Point height: " << coord[2] << "\n";
    // bool result= z_coord<height;
    bool result=(coord[2]-height)<-height_tolerance;
    // if (result){std::cout << "Ground Height: " << height << "\n"; }
    return result;
}

template<typename T> 
bool FootSwingTrajectoryPat<T>::pointTravCheck(Vec3<T> coord,const DMat<int> & idx_map){
    int row_idx_half = idx_map.rows()/2;
    int col_idx_half = idx_map.cols()/2;

    int x_idx = -floor(coord[0]/grid_size) + row_idx_half;
    int y_idx = -floor(coord[1]/grid_size) + col_idx_half;

    int x_idx_selected = x_idx;
    int y_idx_selected = y_idx;

    if(x_idx_selected < 0 || x_idx_selected > idx_map.rows()){
      printf("[WARNING] selected x index is outside of the height map: %d\n", x_idx_selected );
      x_idx_selected = x_idx;
    }
    if(y_idx_selected < 0 || y_idx_selected > idx_map.cols()){
      printf("[WARNING] selected y index is outside of the height map: %d\n", y_idx_selected );
      y_idx_selected = y_idx;
    }
    int traversable=idx_map(x_idx_selected, y_idx_selected);
    if (traversable==0){
      return true;
    }
    else{
      return false;
    }
}

template<typename T>  
int FootSwingTrajectoryPat<T>::legCollisionCheck(ControlFSMData<T>& data,
     const DMat<T> & height_map,int traj_collision_pts,int shin_collision_pts, int leg, T phase, Vec3<T> q0){
      // define some params
      Vec3<T> joints,knee_pos, rel_foot_pos,point;
      // Vec3<T> knee_pos,point;
      T idx;
      bool predict_viz=true;
      bool viz_spheres=true;
      // Vec3<T>& q0=_q0;

      //get joints from IK
      rel_foot_pos=data._stateEstimator->getResult().rBody*(_foot_pos-_abad_pos);
      joints=PatKinematics::ikFoot<T>(*data._pat,rel_foot_pos, q0,leg);
      _q0=joints;

      //Compute knee position using the joints from ik
      knee_pos=_abad_pos+data._stateEstimator->getResult().rBody.transpose() * PatKinematics::fkKnee<T>(*data._pat, joints, leg); //knee pos in world frame
      if (predict_viz){
        data.visualizationData->clear();
        auto* legPrediction = data.visualizationData->addPath();
        legPrediction->num_points = 3;
        legPrediction->color = {0.2,1,0.2,0.5};
        legPrediction->position[0] = {(float)_abad_pos[0],(float)_abad_pos[1],(float)_abad_pos[2]}; //static_cast<Vec3<float>>(_abad_pos);
        legPrediction->position[1] = {(float)knee_pos[0],(float)knee_pos[1],(float)knee_pos[2]};
        legPrediction->position[2] = {(float)_foot_pos[0],(float)_foot_pos[1],(float)_foot_pos[2]};
      }

      for(int j = 0; j <= shin_collision_pts; j++) {   // //check different points in the leg
        idx=(static_cast<T>(j)/static_cast<T>(shin_collision_pts));
        // std::cout<<"shin_idx "<<j<<" | "<<idx<<"\n";
        point=Interpolate::lerp<Vec3<T>>(_foot_pos, knee_pos, idx)-_body_pos_ground; //collision point in body_world frame

        if (viz_spheres){
          SphereVisualization* footstepSphere = data.visualizationData->addSphere();
          footstepSphere->position[0] = point[0]+_body_pos_ground[0];
          footstepSphere->position[1] = point[1]+_body_pos_ground[1];
          footstepSphere->position[2] = point[2]+_body_pos_ground[2];
          footstepSphere->radius = 0.01;
          footstepSphere->color = {0.2, 0, 1, 0.8};
        }
        // std::cout<<"point coord: "<<point<< "\n";
        if (pointCollisionCheck(point,height_map)){  //collision detected
          if(phase>=.75){ //check if it's a touchdown collision
            if (j!=0){//don't do anything if it's the final foot
              return 2;
            }
            // std::cout<<"Final footstep collision\n";
            // std::cout<<"Do nothing\n\n";
          }
          else{ //mid-traj collision
            //  return point+body_pos_ground; //return collision_point in world frame
            // std::cout<<"Detected collision\n";
            // std::cout<<"collision coord: "<<point+body_pos_ground<< "\n";
            //Visualize collision sphere
            SphereVisualization* collisionSphere = data.visualizationData->addSphere();
            collisionSphere->position[0] = point[0]+_body_pos_ground[0];
            collisionSphere->position[1] = point[1]+_body_pos_ground[1];
            collisionSphere->position[2] = point[2]+_body_pos_ground[2];
            collisionSphere->radius = 0.02;
            collisionSphere->color = {1, 0, 0, 0.8};
            
            return 1;
          }
        }
      }
      return 0;

     }

template<typename T> 
void FootSwingTrajectoryPat<T>::moveFootstep(Vec3<T> & footstep, T dist){ //move footstep in the x direction of the footswing frame
  Vec3<T> dX;
  // dX[0]=diff[0];
  // dX[1]=diff[1];
  dX[0]=0.0;
  dX[1]=dist*_side_sign;
  dX[2]=0.0;
  // dX.normalize();
  // footstep+=dX*dist;
  footstep+=rot_z*dX;
}



template<typename T> 
void FootSwingTrajectoryPat<T>::exploreTouchdown(ControlFSMData<T>& data,
     const DMat<T> & height_map, const DMat<int> & idx_map,int traj_collision_pts,int shin_collision_pts, int leg, T phase){ //move footstep in the x direction of the footswing frame
  // while some condition explore footsteps by moving them and checking traversability
  T d_max=.16; //max distance to deviate from
  T dx=.04; //distance to move on every try
  int ctr=1;
  int collision=0;
  bool traversable=false;
  Vec3<T> footstep= _pf;
  std::cout<<"Footstep "<< footstep[0]<<"  "<< footstep[1]<<"  "<< footstep[2] << "\n";
  _abad_pos=initial_abad_pos+diff;
  while(dx*ctr<= d_max){
    moveFootstep(footstep,dx);
    // SphereVisualization* exploreSphere = data.visualizationData->addSphere();
    //         exploreSphere->position[0] = footstep[0];
    //         exploreSphere->position[1] = footstep[1];
    //         exploreSphere->position[2] = footstep[2];
    //         exploreSphere->radius = 0.02;
    //         exploreSphere->color = {1, 0, 0, 0.8};
    std::cout<<"Footstep "<< footstep[0]<<"  "<< footstep[1]<<"  "<< footstep[2] << "\n";
    std::cout<<"Moved footstep by "<< dx*ctr << "\n";
    traversable=pointTravCheck(footstep-_body_pos_ground,idx_map);
    if (traversable){ //check traversability 
      std::cout<<"Traversable!\n";
      //need to update new footstep height
      _foot_pos=footstep;
      collision=legCollisionCheck(data,height_map,traj_collision_pts,shin_collision_pts,leg,phase, _q0);
      if(collision==0){ //successful placement (no collision)
        _pf=footstep;
        std::cout<<"Successful placement\n";
        break; //could return as well
      }
      else {std::cout<<"Collision from moving by "<< dx*ctr << "\n";}
    
    }
    else{std::cout<<"Not traversable\n";}
    ctr++;
  }
}

template<typename T>  
int FootSwingTrajectoryPat<T>::predictTrajCollision(ControlFSMData<T>& data,
     const DMat<T> & height_map,int traj_collision_pts,int shin_collision_pts, int leg, T swingState){
    /*!
   * Checks for trajectory feasability
   * Returns 0 if no collision was predicted
   *         1 if mid collision predicted
   *         2 if touchdown collision was predicted
   */
    // Vec3<T> rel_foot_pos,knee_pos,traj_delta,joints,point,abad_pos;
    // Vec3<T> foot_pos,abad_pos;
    // Vec3<T> foot_pos,abad_pos, rel_foot_pos, joints;
    int collision=0;
    auto seResult = data._stateEstimator->getResult();
    // Vec3<T> collision_point={0, 0, 0};
    bool flag=true;
    T swingTime=1; //arbitrary since we don't care about traj vel/accel
    
    //Get Current Robot information
    Vec3<T> body_pos=seResult.position; //fpr use of height map
    _body_pos_ground= {body_pos[0],body_pos[1],0};//shifted height map frame wiht z=0 on the ground
    // Vec3<T> foot_pos=seResult.position + seResult.rBody.transpose() * (data._pat->getHipLocation(leg) + 
    //                 data._legController->datas[leg].p); //Get foot position in world frame from state estimator
    
    Vec3<T> initial_abad=getInitialAbadPosition(); //Set initial abad_pos in world frame
    _q0=data._legController->datas[leg]->q; //current joints

    //Get Traj information
    Vec3<T> p0=getInitialPosition();
    Vec3<T> pf=getFinalPosition();
    
    T step = (1.f - swingState) / (traj_collision_pts);
    T phase, idx;

    // auto* path_col = data.visualizationData->addPath();
    // path_col->num_points = traj_collision_pts;
    // path_col->color = {0.2,1,0.2,0.5};

    for(int i = 1; i <= traj_collision_pts; i++) { //checks future configuration in the trajectory, 
      //Convert traj_index to phase index (0 to 1)
      phase=swingState + i * step;
      //compute foot position
      computeBezier(phase, swingTime); //compute where the foot will be
      _foot_pos=getPosition(); //foot pos in world frame
      //predict abad_traj in world frame
      _abad_pos= initial_abad+(Interpolate::lerp<Vec3<T>>(p0,pf,phase)-p0); //Not accurate assumption, traj delta w no timing
      //Compute joints through inverse kinematics
      // rel_foot_pos=data._stateEstimator->getResult().rBody*(foot_pos-abad_pos); //foot position in the abad frame, need rot matrix
      // rel_foot_pos=data._stateEstimator->getResult().rBody*(foot_pos-abad_pos);
      // joints=PatKinematics::ikFoot<T>(*data._pat,rel_foot_pos, q0,leg);
      collision=legCollisionCheck(data,height_map,traj_collision_pts,shin_collision_pts,leg,phase, _q0);
      if(collision){
        return collision;
      }
    }
    return 0;
}

template <typename T>
void FootSwingTrajectoryPat<T>::updateTrajectoryBehavior(ControlFSMData<T>& data,
     const DMat<T> & height_map,const DMat<int> & idx_map, int traj_collision_pts,int shin_collision_pts, int leg, T swingState,T timeout, Vec3<T> & footstep){
  bool swing_back=false;
  bool first_time=true;
  int traj_idx;
  int move_ctr=1;
  // int start_search=2;
  int collision=0;
  setTrajectoryType(SWING_DEFAULT);
  if (_pf[2]-_p[2]> .05){// bias first guess towards swinging back
    traj_idx=SWING_BACK;
    setTrajectoryType(SWING_BACK);
    swing_back=true;
    // std::cout<<"I'm swinging back\n";
  }
  
  auto start = chrono::steady_clock::now();
  collision=predictTrajCollision(data,height_map,traj_collision_pts,shin_collision_pts,leg,swingState);
  while(collision){//&& chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now()-start).count()<timeout)
    if(collision==1){ //mid collision
      if (first_time){
        traj_idx=SWING_HIGH;
      }
      else{
        traj_idx++;
        traj_idx=(traj_idx==SWING_BACK && swing_back) ? SWING_FRONT : traj_idx; //skip swing back if you've tried it
        if (traj_idx>=library_size){
          std::cout<<"Out of Trajectories to Try\n";
          setTrajectoryType(SWING_HIGH);
          break;
        }
      }
      setTrajectoryType(traj_idx);
    }

    else if(collision==2){ //touchdown collision
      moveFootstep(footstep, move_ctr);
      std::cout<<"Moved Footstep\n";
    }
    
    if (chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now()-start).count()>timeout){ 
      std::cout<<"Out of Time\n";
      setTrajectoryType(SWING_HIGH);
      break;
    }
    
    // std::cout<<"Predicted Collision of type: "<< collision<<" changing traj to "<< traj_idx<<"\n";
    first_time=false;
    collision=predictTrajCollision(data,height_map,traj_collision_pts,shin_collision_pts,leg,swingState);
  }
  // moveFootstep(footstep_ptr, 1);
  // std::cout<<"Finished updating trajectory\n"; 
}

template <typename T>
void FootSwingTrajectoryPat<T>::updateTouchdown(ControlFSMData<T>& data,
     const DMat<T> & height_map,const DMat<int> & idx_map, int traj_collision_pts,int shin_collision_pts, int leg, T swingState,T timeout, Vec3<T> & footstep){
  int traj_idx;
  int move_ctr=1;
  // int start_search=2;
  int collision=predictTrajCollision(data,height_map,traj_collision_pts,shin_collision_pts,leg,swingState);
  if (collision){
    std::cout<<"Predicted Collision on swing "<< collision<<"   Exploring footsteps\n";
    exploreTouchdown(data, height_map, idx_map, traj_collision_pts, shin_collision_pts, leg, 1);
  }
  
}

template <typename T>
void FootSwingTrajectoryPat<T>::visualize(ControlFSMData<T>& data,T swingState){
      T viz_points=100;
      T swingTime=1;
      // if (_data->_gaitScheduler->gaitData.contactStateScheduled(foot) == 0) {
      T colAct[3] = {1.0,0.0,0.0}; //red
      T colDes[3] = {0.0,0.0,1.0}; //blue
      //
      SphereVisualization* sphere1 = data.visualizationData->addSphere();
      sphere1->position=_p0.template cast<float> ();
      sphere1->radius = 0.05;
      sphere1->color = {0, 0, 1, 0.8};

      SphereVisualization* sphere2 = data.visualizationData->addSphere();
      sphere2->position=wp[1].template cast<float> ();
      sphere2->radius = 0.02;
      sphere2->color = {0, 0, 1, 0.8};

      SphereVisualization* sphere3= data.visualizationData->addSphere();
      sphere3->position=wp[2].template cast<float> ();
      sphere3->radius = 0.02;
      sphere3->color = {0, 0, 1, 0.8};

      SphereVisualization* sphere4 = data.visualizationData->addSphere();
      sphere4->position=_pf.template cast<float> ();
      sphere4->radius = 0.05;
      sphere4->color = {1, 0, 0, 0.8};
      // VisualizeSphere(footSwingTrajectoriesPat[foot].getInitialPosition(), colAct, 0.8);
      // VisualizeSphere(footSwingTrajectoriesPat[foot].getBezierControlPoint1(), colAct, 0.8);//
      // VisualizeSphere(footSwingTrajectoriesPat[foot].getBezierControlPoint2(), colDes, 0.8);
      // VisualizeSphere(footSwingTrajectoriesPat[foot].getFinalPosition(), colDes, 0.8);
      // Add the swing trajectories
      auto* swingTraj = data.visualizationData->addPath();
      swingTraj->num_points = viz_points;
      swingTraj->width=.02;
      // // swingTraj->color = {legColors[3 * foot + 0], legColors[3 * foot + 1], legColors[3 * foot + 2], 0.5};
      if (traj_type == SWING_DEFAULT){
        swingTraj->color = {0,0,1,.5};
      }
      else if(traj_type == SWING_HIGH){
        swingTraj->color = {0,1,1,.5};
      }
      else if(traj_type == SWING_BACK){
        swingTraj->color = {1,0,0,.5};
      }
      else if(traj_type == SWING_FRONT){
        swingTraj->color = {1,1,0,.5};
      }
      else{
        swingTraj->color = {1,1,1,.5};
      }
      
      T step = (1.f - swingState) / viz_points;
      for (int i = 0; i < viz_points; i++) {
        // footSwingTrajectoriesPat[foot].computeSwingTrajectoryBezier(
        computeBezier(swingState + i * step, swingTime);
        swingTraj->position[i] = _p.template cast<float> ();
        // swingTraj->position[i][0] = _p[0];
        // swingTraj->position[i][1] = _p[1];
        // swingTraj->position[i][2] = _p[2];
      }
    // // // }
  }


template class FootSwingTrajectoryPat<double>;
template class FootSwingTrajectoryPat<float>;
