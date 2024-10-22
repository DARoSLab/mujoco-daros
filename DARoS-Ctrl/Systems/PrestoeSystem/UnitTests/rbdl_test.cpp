#include <iostream>
#include <urdf/urdf_parser.h>
#include <urdf/model.h>
#include <rbdl/rbdl.h>
#include <rbdl/urdfreader.h>

#include <Configuration.h>


using namespace dynacore::urdf;
using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;


int main (int argc, char** argv  ){
  RigidBodyDynamics::Model* model_ = new RigidBodyDynamics::Model();
  if (!Addons::URDFReadFromFile 
      (THIS_COM"Systems/prestoe/prestoe_config/prestoe_urdf.urdf", model_, true, true)) {
      //(THIS_COM"Systems/prestoe/prestoe_config/mercury.urdf", model_, true, true)) {
    std::cerr << "Error loading model prestoe.urdf" << std::endl;
    abort();
  }

  return 0;
}
