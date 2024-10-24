#ifndef __PRESTOE_FLOATING_BASE_MODEL_H__
#define __PRESTOE_FLOATING_BASE_MODEL_H__ 

#include <FBModel/FloatingBaseModel.h>
#include <FBModel/parsingURDF.h>
#include <Configuration.h>

//  Foot index
//  1 --- 3
//  |     |
//  2 --- 4
//   \   /
//    | |
//    heel

namespace prestoe_fb_link { // numbering depends on order of how contact points are added
  constexpr size_t rfoot = 12; 
  constexpr size_t rtoe = 13; 
  constexpr size_t lfoot = 19; 
  constexpr size_t ltoe = 20; 
}

namespace prestoe_contact{
  constexpr size_t rheel = 8; 
  constexpr size_t rtoe_1 = 9;
  constexpr size_t rtoe_2 = 10;
  constexpr size_t rtoe_3 = 11;
  constexpr size_t rtoe_4 = 12;

  constexpr size_t lheel = 13; 
  constexpr size_t ltoe_1 = 14;
  constexpr size_t ltoe_2 = 15;
  constexpr size_t ltoe_3 = 16;
  constexpr size_t ltoe_4 = 17;

  constexpr size_t num_foot_contact = 10;
}

/*!
 * Representation of a Prestoe robot's physical properties.
 * Leg
 * 1 0   RIGHT
 * 3 2   RIGHT
 * Arm
 *
 */
template <typename T>
class PrestoeFBModel {
  public:
    static void buildFBModel(FloatingBaseModel<T> & model, bool verbose = false, T gravity = -9.81)
    {
      buildFloatingBaseModelFromURDF(model, 
      THIS_COM"/Systems/PrestoeSystem/Robot/prestoe_urdf.urdf", verbose);

      // Contact setup
      Vec3<T> offset;
      offset.setZero();
      Vec3<T> waistDims(0.2, 0.2, 0.3);
      offset[2] = waistDims[2]/2.0; 
      model.addGroundContactBoxPointsOffset(5, waistDims, offset);

      // right foot contact
      model.addGroundContactPoint(prestoe_fb_link::rfoot, Vec3<T>(-0.055, 0.0, -0.04));
      model.addGroundContactPoint(prestoe_fb_link::rtoe, Vec3<T>(0.075, 0.03, -0.031));
      model.addGroundContactPoint(prestoe_fb_link::rtoe, Vec3<T>(-0.02, 0.03, -0.031));
      model.addGroundContactPoint(prestoe_fb_link::rtoe, Vec3<T>(0.075, -0.03, -0.031));
      model.addGroundContactPoint(prestoe_fb_link::rtoe, Vec3<T>(-0.02, -0.03, -0.031));

      // left foot contact
      model.addGroundContactPoint(prestoe_fb_link::lfoot, Vec3<T>(-0.055, 0.0, -0.04));
      model.addGroundContactPoint(prestoe_fb_link::ltoe, Vec3<T>(0.075, 0.03, -0.031));
      model.addGroundContactPoint(prestoe_fb_link::ltoe, Vec3<T>(-0.02, 0.03, -0.031));
      model.addGroundContactPoint(prestoe_fb_link::ltoe, Vec3<T>(0.075, -0.03, -0.031));
      model.addGroundContactPoint(prestoe_fb_link::ltoe, Vec3<T>(-0.02, -0.03, -0.031));

      Vec3<T> g(0, 0, gravity);
      model.setGravity(g);
    }
};

#endif // LIBBIOMIMETICS_Prestoe_H
