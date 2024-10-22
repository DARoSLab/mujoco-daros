#ifdef MOCAP_BUILD
#include "PatMocapManager.hpp"
#include <unistd.h>
#include <mutex>
template<typename T>
PatMocapManager<T>::PatMocapManager():lcm(getLcmUrl(255)){
}
template<typename T>
PatMocapManager<T>::PatMocapManager(MOCAPData<T>* mocapData):lcm(getLcmUrl(255)){
  _mocapData = mocapData;
  _mocapData->local_poses.resize(NUM_MARKERS);
}
template<typename T>
void PatMocapManager<T>::initialize(){
  // _w_R_m << 0, 0, 1,
  //           -1, 0, 0,
  //           0, 1, 0;
  _w_R_m << 1, 0, 0,
            0, 0, -1,
            0, 1, 0;
  for(int i(0); i<3; i++){
    _velocity_filt.push_back(deriv_lp_filter<float>(2*M_PI*50, 0.002));
  }
  for(int i(0); i<3; ++i){
    body_led0_filter_.push_back(digital_lp_filter<float>(2.*M_PI*50, 0.002));
    body_led1_filter_.push_back(digital_lp_filter<float>(2.*M_PI*50, 0.002));
    body_led2_filter_.push_back(digital_lp_filter<float>(2.*M_PI*50, 0.002));
  }
  if(owl.open(address) <= 0 || owl.initialize() <= 0){

    std::cerr << "Couldn't Open Mocap" << '\n';
    return;

  }
  _healthy_led_list.resize(NUM_MARKERS);
  _led_cond_list.resize(NUM_MARKERS);
  _led_list.resize(NUM_MARKERS);
  _local_poses.resize(NUM_MARKERS);
  for(int i(0); i<NUM_MARKERS; ++i){
    _led_list[i].setZero();
    _local_poses[i].setZero();
  }
  _body_position_est.setZero();
  _body_velocity_est.setZero();
  _body_orientation_est.setZero();
  _R_coord.setIdentity();
  // start streaming
  owl.streaming(1);
  std::cout << "[MOCAP initialization done]" << '\n';
}
template<typename T>
void PatMocapManager<T>::run(){

  const OWL::Event *event = owl.nextEvent(MM_TIMEOUT);
  if(!event) return;
  if(event->type_id() == OWL::Type::ERROR)
    {
      std::cerr << event->name() << ": " << event->str() << std::endl;
      return;
    }
  else if(event->type_id() == OWL::Type::FRAME &&
          event->find("markers", markers) > 0)
    {
        if(_update_body_coord)
        {
            for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++){
              _led_cond_list[m->id] = m->cond;
              if(_led_cond_list[m->id]>0.0)
                _healthy_led_list[m->id] << m->x, m->y, m->z;
            }
            bool update_cond = _led_cond_list[0] > 0.0 &&
                               _led_cond_list[1] > 0.0 &&
                               _led_cond_list[2] > 0.0;

            if(update_cond){
              _R_coord = _GetOrientation(_healthy_led_list[0],
                _healthy_led_list[1],
                _healthy_led_list[2]);

              _offset = _healthy_led_list[0];
              _update_body_coord = false;
              std::cout << "_R_coord: " << _R_coord << '\n';
              std::cout << "Done updating Coordinate!!" << '\n';
            }{
              std::cout << "Waiting For Body Markers to be visible" << '\n';
            }

        }

        else{
            // std::cout << "reading mocap data!!" << '\n';

              for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++){
                _led_cond_list[m->id] = m->cond;
                _led_list[m->id] << m->x, m->y, m->z;

              }

              Vec3<T> local_pos;
              for (int i = 0; i < NUM_MARKERS; ++i) {
                // std::cout << "R_coord: " << _R_coord << '\n';
                _local_poses[i] = POS_SCALE*_R_coord*(_led_list[i] - _offset);
                _mocapData->local_poses[i] = _local_poses[i];
                // std::cout <<"Marker " << i << " pos: " << _mocapData->local_poses[i] << '\n';
              }

              _body_position_est << _local_poses[0];

              // Vec3<T> mocap_pos;
              // for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++){
              //   if(m->cond > 0 && m->id == COM_MARKER_ID){
              //     mocap_pos << m->x, m->y, m->z;
              //     _body_position_est << POS_SCALE*_w_R_m*mocap_pos;
              //     // _body_position_est << POS_SCALE*mocap_pos;
              //
              //     // std::cout << "m time: " << m->time<< '\n';
              //     // pretty_print(_body_position_est, std::cout, "body_position_est: ");
              //     // _body_position_est << POS_SCALE*_w_R_m*_R_coord*( mocap_pos - _offset);
              //     // _body_position_est << POS_SCALE*_R_coord*(mocap_pos - _offset);
              //     // _body_position_est << POS_SCALE*mocap_pos;
              //
              //   }
              //
              // }


        }

  }
  updateCOMVelocity();
  publish_mocap_data();
  // usleep(1000);
}

template<typename T>
void PatMocapManager<T>::publish_mocap_data(){

  for(int i(0); i<3; ++i){
    _mocap_lcm_data.bodyPosition[i] = _body_position_est[i];
    _mocap_lcm_data.bodyVelocity[i] = _body_velocity_est[i];
    _mocap_lcm_data.lf_pos[i] = _local_poses[6][i];
    _mocap_lcm_data.rf_pos[i] = _local_poses[7][i];
  }
  for(int i(0); i<4; ++i)
    _mocap_lcm_data.bodyOrientation[i] = _body_orientation_est[i];

  lcm.publish("pat_mocap_est_info", &_mocap_lcm_data);

}
template<typename T>
void PatMocapManager<T>::updateCOMVelocity(){

  for(int i(0); i<3; i++){
    _velocity_filt[i].input(_body_position_est[i]);
    _body_velocity_est(i) = _velocity_filt[i].output();
  }

}

template<typename T>
Mat3<T> PatMocapManager<T>::_GetOrientation(const Vec3<T> &b0_raw,
        const Vec3<T> &b1_raw,
        const Vec3<T> &b2_raw) {

    Vec3<T> b0, b1, b2;
    //
    for(int i(0); i<3; ++i){
        // body_led0_filter_[i].input(b0_raw[i]);
        // body_led1_filter_[i].input(b1_raw[i]);
        // body_led2_filter_[i].input(b2_raw[i]);
        //
        // b0[i] = body_led0_filter_[i].output();
        // b1[i] = body_led1_filter_[i].output();
        // b2[i] = body_led2_filter_[i].output();
        b0[i] = b0_raw[i];
        b1[i] = b1_raw[i];
        b2[i] = b2_raw[i];
    }

    Vec3<T> normal;
    normal = (b2 - b0).cross( (b1 - b0) );
    // normal = (b1 - b0).cross( (b2 - b0) );
    normal /= sqrt(normal[0]*normal[0] + normal[1]*normal[1] +
            normal[2] * normal[2]);
    Vec3<T> x_coord, y_coord, z_coord;
    x_coord = normal;
    // Y
    // y_coord = b1-b2;
    y_coord = b2-b1;
    y_coord /= sqrt(y_coord[0]* y_coord[0] + y_coord[1]*y_coord[1] + y_coord[2]*y_coord[2]);
    // Z
    z_coord = x_coord.cross(y_coord);
    z_coord /= sqrt(z_coord[0]*z_coord[0] + z_coord[1]*z_coord[1] +
            z_coord[2] * z_coord[2]);
    Mat3<T> R(3, 3);
    R << x_coord[0], y_coord[0], z_coord[0],
    x_coord[1], y_coord[1], z_coord[1],
    x_coord[2], y_coord[2], z_coord[2];

    return R.transpose();
}
template class PatMocapManager<double>;
template class PatMocapManager<float>;
#endif
