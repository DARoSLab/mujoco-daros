/*!
 * @file JPosInitializer.h
 * @brief Controller to initialize the position of the legs on power-on
 */

#ifndef GUIDE_DOG_JPOS_INITIALIZER
#define GUIDE_DOG_JPOS_INITIALIZER

#include "LegControllerPat.h"
#include <robots/PatBiped.h>
#include <Utilities/BSplineBasic.h>

/*!
 * Controller to initialize the position of the legs on power-on
 */
template <typename T>
class JPosInitializerPat {
 public:
  JPosInitializerPat(T end_time, float dt);
  ~JPosInitializerPat();

  bool IsInitialized(LegControllerPat<T>*);

 private:
  void _UpdateParam();
  void _UpdateInitial(const LegControllerPat<T>* ctrl);
  bool _b_first_visit;
  T _end_time;
  T _curr_time;
  T _dt;

  std::vector<T> _ini_jpos;
  std::vector<T> _target_jpos;
  std::vector<T> _mid_jpos;

  BS_Basic<T, pat_biped::num_act_joints, 3, 1, 2, 2> _jpos_trj;
};
#endif
