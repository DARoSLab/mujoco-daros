#pragma once
#include <eigen3/Eigen/Core>
#include <vector>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <casadi/casadi.hpp>
using namespace std;
using namespace casadi;
class params{
    public:
        params();
        ~params(){}

        int pred_hor = 10;


        DM x_des = DM({0, 0, 0.775});
        DM v_des = DM({0, 0, 0});
        DM w_des = DM({0, 0, 0});
        DM R_des = DM({1, 0, 0, 0, 1, 0, 0, 0, 1});

        DM x_init_condition = DM({0, 0, 0.775});
        DM xd_init_condition = DM({0, 0, 0});
        DM R_init_condition = DM({1, 0, 0, 0, 1, 0, 0, 0, 1});
        DM w_init_condition = DM({0, 0, 0});
   
   
};