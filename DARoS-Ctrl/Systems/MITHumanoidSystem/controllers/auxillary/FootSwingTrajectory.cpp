/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#include <Utilities/Interpolation.h>
#include "FootSwingTrajectory.h"
#include <iostream>
/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */
template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swingTime) {
  // std::cout<<"phase: \n"<<phase<<std::endl;
  // std::cout<<"swingTime: \n"<<swingTime<<std::endl;

  _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
  _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase) / swingTime;
  _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);

  T zp, zv, za;

  if(phase < T(0.5)) {
    zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _height, phase * 2);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
  } else {
    // std::cout<< "_p0[2] + _height \n" << _p0[2] + _height << std::endl;
    // std::cout<< "_pf[2] \n" << _pf[2] << std::endl;
    // std::cout<< "phase * 2 - 1 \n" << phase * 2 - 1 << std::endl;

    zp = Interpolate::cubicBezier<T>(_p0[2] + _height, _pf[2], phase * 2 - 1);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
    // std::cout<<"za \n"<<za<<std::endl;
  }

  //T middle_phase = 0.4;
  //if(phase < middle_phase) {
    //T adjusted_phase = phase/middle_phase;
    //zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _height, adjusted_phase);
    //zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _height, adjusted_phase) / middle_phase / swingTime;
    //za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _height, adjusted_phase) /middle_phase /middle_phase / (swingTime * swingTime);
  //} else {
    //T adjusted_phase = (phase-middle_phase)/(1.-middle_phase);
    //zp = Interpolate::cubicBezier<T>(_p0[2] + _height, _pf[2], adjusted_phase);
    //zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _height, _pf[2], adjusted_phase) / (1-middle_phase) / swingTime;
    //za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _height, _pf[2], adjusted_phase) / (1-middle_phase)/ (1-middle_phase) / (swingTime * swingTime);
  //}

  _p[2] = zp;
  _v[2] = zv;
  _a[2] = za;

  // std::cout<<"init pos: \n"<<_p0<<std::endl;
  // std::cout<<"final pos: \n"<<_pf<<std::endl;
}

template class FootSwingTrajectory<double>;
template class FootSwingTrajectory<float>;
