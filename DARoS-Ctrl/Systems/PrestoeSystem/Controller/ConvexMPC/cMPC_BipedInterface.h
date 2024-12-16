#ifndef _convexmpc_biped_interface
#define _convexmpc_biped_interface
// #define K_MAX_GAIT_SEGMENTS 36

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include <eigen3/Eigen/Dense>
#include "cMPC_types.h"
struct problem_setup
{
  float dt;
  float mu;
  float f_max;
  int horizon;
  bool general_form = false;
};

struct update_data_t
{
  float p[3];
  float v[3];
  float q[4];
  float w[3];
  float r[3*NUM_CONTACTS*K_MAX_GAIT_SEGMENTS];
  float yaw;
  float weights[12];
  float traj[12*K_MAX_GAIT_SEGMENTS];
  float dt_segments[K_MAX_GAIT_SEGMENTS];
  float alpha;
  unsigned char gait[K_MAX_GAIT_SEGMENTS];
  unsigned char hack_pad[1000];
  int max_iterations;
  double rho, sigma, solver_alpha, terminate;
  int use_jcqp;
  float x_drag;
  float I_vec[3];
  float m;
};
//General form variable dt and foot_step location
EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max, bool general_form=false);
EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);
// EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait, double* dt_segment);
EXTERNC double get_solution(int index);
EXTERNC mfp* get_mpc_solution();
// EXTERNC void get_mpc_solution(mfp* u_star);
EXTERNC void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha, int* gait);

EXTERNC void update_problem_data_var_segments(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha, int* gait, float* dt_segments, float* I_vec, float m);


void update_x_drag(float x_drag);

// Eigen::Matrix<float,13,13> get_A();
// Eigen::Matrix<float,13,3*NUM_CONTACTS> get_B();

void get_A(Eigen::Matrix<fpt,13,13> &A);
void get_B(Eigen::Matrix<fpt,13,3*NUM_CONTACTS> &B);

void get_mpc_traj(Eigen::Matrix<fpt,Eigen::Dynamic, 1> & X_mpc, const int horizon);
void get_mpc_Aqpx0(Eigen::Matrix<fpt,Eigen::Dynamic, 1> & Aqp_x0);
void get_mpc_x0(Eigen::Matrix<fpt,13,1> &x0);
void get_mpc_BqpU(Eigen::Matrix<fpt,Eigen::Dynamic, 1> & BqpU, const int horizon);
void getXref(Eigen::Matrix<fpt, Eigen::Dynamic, 1> &Xref);
void get_mpc_Aqp(Eigen::Matrix<fpt, Eigen::Dynamic,13> &Aqp);
void get_mpc_Bqp(Eigen::Matrix<fpt, Eigen::Dynamic,Eigen::Dynamic> &Bqp);

void get_mpc_Ub(Eigen::Matrix<fpt,Eigen::Dynamic,1> &Ub);
void get_mpc_Lb(Eigen::Matrix<fpt,Eigen::Dynamic,1> &Ul);
void get_mpc_C(Eigen::Matrix<fpt,Eigen::Dynamic,Eigen::Dynamic> &C);
void get_mpc_g(Eigen::Matrix<fpt,Eigen::Dynamic,1> &g);
void get_mpc_H(Eigen::Matrix<fpt,Eigen::Dynamic,Eigen::Dynamic> &H);
// Eigen::Matrix<fpt, Eigen::Dynamic, 1> getXref();
// Eigen::Matrix<fpt, Eigen::Dynamic,13> get_mpc_Aqp();
// Eigen::Matrix<fpt, Eigen::Dynamic,Eigen::Dynamic> get_mpc_Bqp();
#endif
