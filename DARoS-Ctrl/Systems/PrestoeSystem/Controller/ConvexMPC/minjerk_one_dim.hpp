#ifndef __MINJERK_ONE_DIM_H__
#define __MINJERK_ONE_DIM_H__
#include "cppTypes.h"
#include <iostream>

// Implements the polynomial:
// x(t) = a0 + a_1*t + a_2*t^2 + a_3*t^3 + a_4*t^4 + a_5*t^5
// Needs boundary conditions x(to), xdot(to), xddot(to), x(tf), xdot(tf), xddot(tf)
// 		where to and tf are the initial and final times respectively. Also, tf > to.

class MinJerk_OneDimension{
public:
	// Constructors
	MinJerk_OneDimension();
	MinJerk_OneDimension(const Vec3<float> & init, const Vec3<float> & end,
						 const double & time_start, const double & time_end);

	void setParams(const Vec3<float> & init, const Vec3<float> & end,
 			  const double & time_start, const double & time_end);

	// Functions
	void test_func();

	void getPos(const double & time, double & pos);
	void getVel(const double & time, double & vel);
	void getAcc(const double & time, double & acc);

	void printParameters();

	// Destructor
	~MinJerk_OneDimension();


private:
	DMat<float> C_mat; // Matrix of Coefficients
	DMat<float> C_mat_inv;  // Inverse of Matrix of Coefficients
	DVec<float> a_coeffs;   // mininum jerk coeffs. a = [a0, a1, a2, a3, a4, a5, a6];
	DVec<float> bound_cond; // boundary conditions x_b = [ x(to), xdot(to), xddot(to), x(tf), xdot(tf), xddot(tf)]

	Vec3<float> init_cond;  // initial pos, vel, acceleration
	Vec3<float> end_cond;   // final pos, vel, acceleration
	double to; // Starting time
	double tf; // Ending time

	void Initialization();

	// Compute the coefficients
	void compute_coeffs();


};

#endif
