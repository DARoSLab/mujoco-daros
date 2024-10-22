#ifdef MOCAP_BUILD
#ifndef PAT_MOCAP_MANAGER_H
#define PAT_MOCAP_MANAGER_H

#include <lcm-cpp.hpp>
#include <thread>
#include <Utilities/utilities.h>
#include "../../third-parties/phase_space/owl.hpp"
#include <state_machine/ControlFSMData.h>
#include "pat_mocap_est_lcmt.hpp"
#include <PatSystem.hpp>
#include <lcm/lcm-cpp.hpp>
#include <Utilities/filters.h>
#define Z_OFFSET 0.127
#define NUM_MARKERS 8
// #define MM_TIMEOUT 1000 //1MS
#define MM_TIMEOUT 1 //1MS
// #define COM_MARKER_ID 5
// #define MARKER_0 5
// #define MARKER_1 3
// #define MARKER_2 7
#define COM_MARKER_ID 0
#define MARKER_0 0
#define MARKER_1 1
#define MARKER_2 2
#define POS_SCALE  0.001


template<typename T>
class PatMocapManager{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PatMocapManager();
    PatMocapManager(MOCAPData<T>* mocapData);
    void run();
    void initialize();
    ~PatMocapManager();
    Vec3<T> _body_position_est;
    Vec3<T> _body_velocity_est;

  private:
    PatSystem<T>* _pat_system;
    MoCapData<float>* _mocap_data;
    lcm::LCM lcm;
    pat_mocap_est_lcmt _mocap_lcm_data;
    std::vector<deriv_lp_filter<float>> _velocity_filt;
    std::vector<digital_lp_filter<float>> body_led0_filter_;
    std::vector<digital_lp_filter<float>> body_led1_filter_;
    std::vector<digital_lp_filter<float>> body_led2_filter_;
    std::vector<Vec3<T>> _healthy_led_list;
    std::vector<float> _led_cond_list;
    std::vector<Vec3<T>> _led_list;
    std::vector<Vec3<T>> _local_poses;

    std::thread _pat_mocap_thread;
    OWL::Context owl;
    OWL::Markers markers;
    OWL::Marker _com_marker;
    OWL::Rigids rigids;
    std::string address = "192.168.1.230";
    Vec3<T> _body_position_local_est;
    Vec4<T> _body_orientation_est;

    Vec3<T> _offset;
    Mat3<T> _R_coord;

    Mat3<T> _w_R_m;
    bool first_visit = true;
    bool _update_body_coord = true;

    MOCAPData<T>* _mocapData;
    void MoCapThread();
    void updateCOMVelocity();
    void publish_mocap_data();
    Mat3<T> _GetOrientation(const Vec3<T> &b0_raw,
            const Vec3<T> &b1_raw,
            const Vec3<T> &b2_raw);
};
#endif
#endif //MOCAP_BUILD
