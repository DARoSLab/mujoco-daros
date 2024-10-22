/*!
 * @file JPosInitializerPat.cpp
 * @brief Controller to initialize the position of the legs on power-on
 */

#include "JPosInitializerPat.h"
#include <Utilities/pretty_print.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Configuration.h>

template <typename T>
JPosInitializerPat<T>::JPosInitializerPat(T end_time, float dt)
    : _b_first_visit(true),
      _end_time(end_time),
      _curr_time(0.),
      _dt(dt),
      _ini_jpos(pat_biped::num_act_joints) {
  _UpdateParam();
}

template <typename T>
JPosInitializerPat<T>::~JPosInitializerPat() {}

template <typename T>
bool JPosInitializerPat<T>::IsInitialized(LegControllerPat<T>* ctrl) {
  _curr_time += _dt;
  // Initial Setup
  if (_b_first_visit) {
    _UpdateInitial(ctrl);
    _b_first_visit = false;
  }

  // Smooth movement
  if (_curr_time < _end_time) {
    T jpos[pat_biped::num_act_joints];
    _jpos_trj.getCurvePoint(_curr_time, jpos);

    // pretty_print(jpos, "jpos_cmd", pat_biped::num_act_joints);

    for (size_t leg(0); leg < pat_biped::num_legs; ++leg) {
      for (size_t jidx(0); jidx < pat_biped::num_legs_joint; ++jidx) {
        ctrl->commands[leg].tauFeedForward[jidx] = 0.;
        ctrl->commands[leg].qDes[jidx] = jpos[3 * leg + jidx];
        ctrl->commands[leg].qdDes[jidx] = 0.;
        ctrl->commands[leg].kpJoint(jidx, jidx) = 5.;
        ctrl->commands[leg].kdJoint(jidx, jidx) = 0.1;
      }
    }
    return false;
  }
  return true;
}

template <typename T>
void JPosInitializerPat<T>::_UpdateInitial(const LegControllerPat<T>* ctrl) {
  T ini[3 * pat_biped::num_act_joints];
  T fin[3 * pat_biped::num_act_joints];
  T** mid = new T*[1];
  mid[0] = new T[pat_biped::num_act_joints];

  for (size_t i(pat_biped::num_act_joints); i < 3 * pat_biped::num_act_joints; ++i) {
    ini[i] = 0.;
    fin[i] = 0.;
  }

  for (size_t leg(0); leg < pat_biped::num_legs; ++leg) {
    for (int jidx(0); jidx < 3; ++jidx) {
      ini[3 * leg + jidx] = ctrl->datas[leg]->q[jidx];
      _ini_jpos[3 * leg + jidx] = ctrl->datas[leg]->q[jidx];
    }
  }

  //pretty_print(_mid_jpos, "mid jpos");
  for (size_t i(0); i < pat_biped::num_act_joints; ++i) {
    fin[i] = _target_jpos[i];
    mid[0][i] = _mid_jpos[i];
  }
  _jpos_trj.SetParam(ini, fin, mid, _end_time);

  delete[] mid[0];
  delete[] mid;
}

template <typename T>
void JPosInitializerPat<T>::_UpdateParam() {
  ParamHandler handler(THIS_COM "config/initial_jpos_ctrl_pat.yaml");
  handler.getVector<T>("target_jpos", _target_jpos);
  handler.getVector<T>("mid_jpos", _mid_jpos);
}

template class JPosInitializerPat<double>;
template class JPosInitializerPat<float>;
