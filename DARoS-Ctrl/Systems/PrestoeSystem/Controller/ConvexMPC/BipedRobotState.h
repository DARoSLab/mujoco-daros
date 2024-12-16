#ifndef Biped_RobotState
#define Biped_RobotState

#include <eigen3/Eigen/Dense>
#include "cMPC_types.h"


using Eigen::Matrix;
using Eigen::Quaternionf;

class BipedRobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        void set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt yaw_, flt* I_vec, flt mass, int horizon);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w;
        Matrix<fpt,3,NUM_CONTACTS> r_feet;
        Matrix<fpt,3,NUM_CONTACTS> r_feet_full[K_MAX_GAIT_SEGMENTS];
        Matrix<fpt,3,3> R;
        Matrix<fpt,3,3> R_yaw;
        Matrix<fpt,3,3> I_body;
        Quaternionf q;
        fpt yaw;
        fpt m;
};
#endif
