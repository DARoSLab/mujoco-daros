#ifndef _convexmpc_biped_interface
#define _convexmpc_biped_interface
#define K_MAX_GAIT_SEGMENTS 36


#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include <eigen3/Eigen/Dense>

struct problem_setup
{
  float dt;
  float mu;
  float f_max;
  int horizon;
};

struct update_data_t
{
  float p[3];
  float v[3];
  float q[4];
  float w[3];
  float r[12];
  float yaw;
  float weights[12];
  float traj[12*K_MAX_GAIT_SEGMENTS];
  float alpha;
  unsigned char gait[K_MAX_GAIT_SEGMENTS];
  unsigned char hack_pad[1000];
  int max_iterations;
  double rho, sigma, solver_alpha, terminate;
  int use_jcqp;
  float x_drag;
};

EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max);
EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);
EXTERNC double get_solution(int index);
EXTERNC void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha, int* gait);
void update_x_drag(float x_drag);

Eigen::Matrix<float,13,13> get_A();
Eigen::Matrix<float,13,12> get_B();

#endif