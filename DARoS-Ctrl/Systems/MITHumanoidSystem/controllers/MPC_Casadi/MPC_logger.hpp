#ifndef MPCLOGGER_H
#define MPCLOGGER_H

#include <unistd.h>
#include <Eigen/Core>
#include <iostream>
#include <casadi/casadi.hpp>
#include "optimizeStand.hpp"
#include <tello_MPC_info_lcmt.hpp>
#include <lcm/lcm-cpp.hpp>
using namespace casadi;
using namespace std;

class MPC_logger{
    public:
        MPC_logger();
        ~MPC_logger(){}
        void send_info(MPC_result info); 

        lcm::LCM _lcm_cart;
};




#endif
