#include "rbdl_wrapper.h"
#include <rbdl/urdfreader.h>

using namespace RigidBodyDynamics;


rbdl_wrapper::rbdl_wrapper(const char* urdf_file, bool b_floatingbase, bool verbose){
  _model = new RigidBodyDynamics::Model();

  if (!Addons::URDFReadFromFile(urdf_file, _model, b_floatingbase, verbose)) {
    std::cerr << "[RBDL Wrapper] Error loading model" << std::endl;
    abort();
  }
}
