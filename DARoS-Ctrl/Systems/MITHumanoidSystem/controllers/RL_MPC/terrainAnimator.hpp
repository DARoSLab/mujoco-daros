#ifndef TERRAINANIMATEOPTI_H
#define TERRAINANIMATEOPTI_H

#include <unistd.h>
#include <Eigen/Core>
#include <iostream>
#include <terrain_lcmt.hpp>
#include <lcm/lcm-cpp.hpp>
using namespace std;
using namespace Eigen;

class terrainAnimator{
    public:
        terrainAnimator();
        ~terrainAnimator(){}

        lcm::LCM _lcm_cart;
        void send_terrain(VectorXd terrain);
};


#endif
