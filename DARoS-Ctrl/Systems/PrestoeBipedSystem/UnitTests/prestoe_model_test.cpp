#include <iostream>
#include <rbdl/rbdl_wrapper.h>
#include <Configuration.h>
#include <urdf/model.h>
#include <urdf/urdf_parser.h>

#include <assert.h>
#include <map>
#include <stack>


#include <pretty_print.h>
#include <Utilities/spatial.h>
#include <Utilities/orientation_tools.h>
#include <Utilities/SpatialInertia.h>

#include <FBModel/FloatingBaseModel.h>

using namespace std;
using namespace ori;
using namespace spatial;


typedef std::shared_ptr<dynacore::urdf::Link> LinkPtr;
typedef const std::shared_ptr<const dynacore::urdf::Link> ConstLinkPtr;
typedef std::shared_ptr<dynacore::urdf::Joint> JointPtr;
typedef std::shared_ptr<dynacore::urdf::ModelInterface> ModelPtr;


typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr > URDFLinkMap;
typedef map<string, JointPtr > URDFJointMap;



int main (int argc, char** argv  ){
  bool verbose(true);
  rbdl_wrapper rbdl_model(THIS_COM"Systems/prestoe/prestoe_config/prestoe_urdf.urdf",true, verbose);
  std::shared_ptr<dynacore::urdf::ModelInterface> urdf_model;
  urdf_model = dynacore::urdf::parseURDFFile(THIS_COM"Systems/prestoe/prestoe_config/prestoe_urdf.urdf", verbose);


  // floating base model
  FloatingBaseModel<double> daros_model;

  URDFLinkMap link_map;
  link_map = urdf_model->links_;

  URDFJointMap joint_map;
  joint_map = urdf_model->joints_;
  vector<string> joint_names;

  // Holds the links that we are processing in our depth first traversal with the top element being the current link.
  stack<LinkPtr > link_stack;
  // Holds the child joint index of the current link
  stack<int> joint_index_stack;
  // add the bodies in a depth-first order of the model tree
  link_stack.push (link_map[(urdf_model->getRoot()->name)]);

  ConstLinkPtr& root = urdf_model->getRoot ();
  Vec3<double> root_inertial_rpy;
  Vec3<double> root_inertial_position;
  Mat3<double> root_inertial_inertia;
  double root_inertial_mass;

  if (root->inertial) {
    root_inertial_mass = root->inertial->mass;

    root_inertial_position << 
      root->inertial->origin.position.x,
      root->inertial->origin.position.y,
      root->inertial->origin.position.z;

    root_inertial_inertia(0,0) = root->inertial->ixx;
    root_inertial_inertia(0,1) = root->inertial->ixy;
    root_inertial_inertia(0,2) = root->inertial->ixz;

    root_inertial_inertia(1,0) = root->inertial->ixy;
    root_inertial_inertia(1,1) = root->inertial->iyy;
    root_inertial_inertia(1,2) = root->inertial->iyz;

    root_inertial_inertia(2,0) = root->inertial->ixz;
    root_inertial_inertia(2,1) = root->inertial->iyz;
    root_inertial_inertia(2,2) = root->inertial->izz;

    root->inertial->origin.rotation.getRPY (root_inertial_rpy[0], root_inertial_rpy[1], root_inertial_rpy[2]);
  }
  SpatialInertia<double> root_inertia(root_inertial_mass, root_inertial_position, root_inertial_inertia);
  daros_model.addBase(root_inertia, urdf_model->getRoot()->name);
  const int baseID = 5;
  int bodyID = baseID;

  // depth first traversal: push the first child onto our joint_index_stack
  joint_index_stack.push(0);

  while(link_stack.size() > 0){
    LinkPtr cur_link = link_stack.top();
    unsigned int joint_idx = joint_index_stack.top();

    // Add any child bodies and increment current joint index if we still have child joints to process.
    if (joint_idx < cur_link->child_joints.size()) {
      JointPtr cur_joint = cur_link->child_joints[joint_idx];

      // increment joint index
      joint_index_stack.pop();
      joint_index_stack.push (joint_idx + 1);

      link_stack.push (link_map[cur_joint->child_link_name]);
      joint_index_stack.push(0);

      if (verbose) {
        for (unsigned int i = 1; i < joint_index_stack.size() - 1; i++) {
          cout << "  ";
        }
        cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name << "' type = " << cur_joint->type << endl;
      }

      joint_names.push_back(cur_joint->name);
    } else {
      link_stack.pop();
      joint_index_stack.pop();
    }
  }


  for (size_t j(0); j < joint_names.size(); ++j) {
    JointPtr urdf_joint = joint_map[joint_names[j]];
    LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
    LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

   // create the joint
    JointType j_type;
    CoordinateAxis j_axis;

    if (urdf_joint->type == dynacore::urdf::Joint::REVOLUTE || urdf_joint->type == dynacore::urdf::Joint::CONTINUOUS) {
      j_type = JointType::Revolute;
      if(urdf_joint->axis.x == 1){ j_axis = CoordinateAxis::X; }
      else if(urdf_joint->axis.y == 1){ j_axis = CoordinateAxis::Y; }
      else if(urdf_joint->axis.z == 1){ j_axis = CoordinateAxis::Z; }
      else{ 
        cerr<<"Joint axis is not along x,y, or z\n"; 
        exit(0);
      }

    } else if (urdf_joint->type == dynacore::urdf::Joint::PRISMATIC) {
      j_type = JointType::Prismatic;
      if(urdf_joint->axis.x == 1){ j_axis = CoordinateAxis::X; }
      else if(urdf_joint->axis.y == 1){ j_axis = CoordinateAxis::Y; }
      else if(urdf_joint->axis.z == 1){ j_axis = CoordinateAxis::Z; }
      else{ 
        cerr<<"Joint axis is not along x,y, or z\n"; 
        exit(0);
      }

    } else if (urdf_joint->type == dynacore::urdf::Joint::FIXED) {
      std::cout<<"Fixed joint: "<< urdf_joint->name<<std::endl;
      continue;
    }
    // determine where to add the current joint and child body
    unsigned int parent_id = 0;

    if (urdf_parent->name != "base_link") {
      parent_id = daros_model.getBodyID(urdf_parent->name.c_str());
    }

    if (parent_id == std::numeric_limits<unsigned int>::max()){
      cerr << "Error while processing joint '" << urdf_joint->name
        << "': parent link '" << urdf_parent->name
        << "' could not be found." << endl;
    }
 
    // compute the joint transformation
    Vec3<double> joint_rpy;
    Vec3<double> joint_translation;
    urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
    joint_translation <<
      urdf_joint->parent_to_joint_origin_transform.position.x,
      urdf_joint->parent_to_joint_origin_transform.position.y,
      urdf_joint->parent_to_joint_origin_transform.position.z;

    // assemble the body
    Vec3<double> link_inertial_position;
    Vec3<double> link_inertial_rpy;
    Mat3<double> link_inertial_inertia = Mat3<double>::Zero();
    double link_inertial_mass = 0.;

    // but only if we actually have inertial data
    if (urdf_child->inertial) {
      link_inertial_mass = urdf_child->inertial->mass;

      link_inertial_position <<
        urdf_child->inertial->origin.position.x,
        urdf_child->inertial->origin.position.y,
        urdf_child->inertial->origin.position.z;

      urdf_child->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

      link_inertial_inertia(0,0) = urdf_child->inertial->ixx;
      link_inertial_inertia(0,1) = urdf_child->inertial->ixy;
      link_inertial_inertia(0,2) = urdf_child->inertial->ixz;

      link_inertial_inertia(1,0) = urdf_child->inertial->ixy;
      link_inertial_inertia(1,1) = urdf_child->inertial->iyy;
      link_inertial_inertia(1,2) = urdf_child->inertial->iyz;

      link_inertial_inertia(2,0) = urdf_child->inertial->ixz;
      link_inertial_inertia(2,1) = urdf_child->inertial->iyz;
      link_inertial_inertia(2,2) = urdf_child->inertial->izz;
    }

    SpatialInertia<double> link_inertia(link_inertial_mass, link_inertial_position, link_inertial_inertia);
    Mat3<double> smallRotorRotationalInertiaZ;
    smallRotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 33;
    smallRotorRotationalInertiaZ = 1e-6 * smallRotorRotationalInertiaZ;
    SpatialInertia<double> rotor_inertia_dummy(0.1e-5, Vec3<double>::Zero(), smallRotorRotationalInertiaZ);
    int gearRatio(9);

    Mat3<double> I3 = Mat3<double>::Identity();
    Mat6<double> Xtree = createSXform(ori::rpyToRotMat(joint_rpy), joint_translation);
    Mat6<double> Xrotor = createSXform(I3, Vec3<double>::Zero());

    if (verbose) {
      printf("----------------------------------------------------------------------\n");
      cout << "+ Adding Body: "    << urdf_child->name << endl;
      cout << "  parent_id  : "    << parent_id << endl;
      cout << "  joint frame: \n"    << Xtree<< endl;
      cout << "  rotor frame:\n "    << Xrotor<< endl;
      cout << "  body inertia: \n" << link_inertia.getInertia() << endl;
      printf("----------------------------------------------------------------------\n");
    }
    daros_model.addBody(link_inertia, rotor_inertia_dummy, gearRatio, parent_id, 
        j_type, j_axis, Xtree, Xrotor, urdf_child->name);
  }

  return 0;
}
