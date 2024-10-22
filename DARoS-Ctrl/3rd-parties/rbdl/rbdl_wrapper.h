#ifndef RBDL_WRAPPER_H
#define RBDL_WRAPPER_H

#include <rbdl/rbdl_config.h>
#include <rbdl/rbdl.h>


class rbdl_wrapper{
  public:
    rbdl_wrapper(const char* urdf_file, bool b_floatingbase, bool verbose = false);
    ~rbdl_wrapper(){}

    RigidBodyDynamics::Model* _model; 
};
#endif
