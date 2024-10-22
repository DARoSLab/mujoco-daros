#include <iostream>
#include <urdf/urdf_parser.h>
#include <urdf/model.h>
#include <Configuration.h>


using namespace dynacore::urdf;

int main (int argc, char** argv  ){
  std::shared_ptr<dynacore::urdf::ModelInterface> model;
  model = parseURDFFile(THIS_COM"Systems/prestoe/prestoe_config/prestoe_urdf.urdf", true);

  return 0;
}
