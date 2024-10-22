/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */


#include "LiftSwingTrajectory.hpp"
#include <Utilities/Interpolation.h>
#include<iostream>
/*!
 * Compute foot swing trajectory with two cubic bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param alpha : proportion of lift phase (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */
template <typename T>
void LiftSwingTrajectory<T>::computeLiftSwingTrajectory(T phase, T alpha, T swingTime) {
  if(phase < alpha) { //lift
    // std::cout << "_p0: " << _p0<< '\n';
    // std::cout << "_pmid: " << _p_mid<< '\n';
    // std::cout << "phase: " <<phase<< '\n';
    // std::cout << "t: " <<phase/alpha<< '\n';


    _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _p_mid, phase/alpha);
    _pdot = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _p_mid, phase) / (alpha*swingTime);
    _pddot = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _p_mid, phase) / (alpha*alpha*swingTime*swingTime);


  }
  else{//step

    // std::cout << "_pmid: " << _p_mid<< '\n';
    // std::cout << "_pf: " << _pf<< '\n';
    // std::cout << "phase: " <<phase<< '\n';
    // std::cout << "t: " <<(phase-alpha)/(1.0-alpha)<< '\n';

    _p = Interpolate::cubicBezier<Vec3<T>>(_p_mid, _pf, (phase-alpha)/(1.0-alpha));
    _pdot = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p_mid, _pf, (phase-alpha)/(1.0-alpha)) / ((1-alpha)*swingTime);
    _pddot = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p_mid, _pf, (phase-alpha)/(1.0-alpha)) / ((1-alpha)*(1-alpha)*swingTime*swingTime);

    // std::cout << "_p: " <<_p.transpose()*_p<< '\n';

    }
}

template class LiftSwingTrajectory<double>;
template class LiftSwingTrajectory<float>;
