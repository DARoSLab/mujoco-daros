#pragma once
#include <casadi/casadi.hpp>
#include "cppTypes.h"
#include <Eigen/Core>
#include <TelloParameters.h>

using namespace std;

using namespace Eigen;


Eigen::MatrixXd generate_contact_cycle(TelloParameters* p, int i);

