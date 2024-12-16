#include "cMPC_BipedInterface.h"
#include "cMPC_types.h"
#include "Solver_cMPC.h"
#include <eigen3/Eigen/Dense>
#include <pthread.h>
#include <stdio.h>
#include <string.h>


problem_setup problem_configuration;
u8 gait_data[K_MAX_GAIT_SEGMENTS];
pthread_mutex_t problem_cfg_mt;
pthread_mutex_t update_mt;
update_data_t update;
pthread_t solve_thread;

u8 first_run = 1;

void initialize_mpc()
{
  //printf("Initializing MPC!\n");
  if(pthread_mutex_init(&problem_cfg_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize problem configuration mutex.\n");

  if(pthread_mutex_init(&update_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize update data mutex.\n");

#ifdef K_DEBUG
  printf("[MPC] Debugging enabled.\n");
    printf("[MPC] Size of problem setup struct: %ld bytes.\n", sizeof(problem_setup));
    printf("      Size of problem update struct: %ld bytes.\n",sizeof(update_data_t));
    printf("      Size of MATLAB floating point type: %ld bytes.\n",sizeof(mfp));
    printf("      Size of flt: %ld bytes.\n",sizeof(flt));
#else
  //printf("[MPC] Debugging disabled.\n");
#endif
}

void setup_problem(double dt, int horizon, double mu, double f_max, bool general_form)
{
  //mu = 0.6;
  if(first_run)
  {
    first_run = false;
    initialize_mpc();
  }

#ifdef K_DEBUG
  printf("[MPC] Got new problem configuration!\n");
    printf("[MPC] Prediction horizon length: %d\n      Force limit: %.3f, friction %.3f\n      dt: %.3f\n",
            horizon,f_max,mu,dt);
#endif

  //pthread_mutex_lock(&problem_cfg_mt);

  problem_configuration.horizon = horizon;
  problem_configuration.f_max = f_max;
  problem_configuration.mu = mu;
  problem_configuration.dt = dt;
  problem_configuration.general_form = general_form;

  //pthread_mutex_unlock(&problem_cfg_mt);
  resize_qp_mats(horizon);
}

//inline to motivate gcc to unroll the loop in here.
inline void mfp_to_flt(flt* dst, mfp* src, s32 n_items)
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}

inline void mint_to_u8(u8* dst, mint* src, s32 n_items)
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}

int has_solved = 0;

//void *call_solve(void* ptr)
//{
//  solve_mpc(&update, &problem_configuration);
//}
//safely copies problem data and starts the solver
void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait)
{
  mfp_to_flt(update.p,p,3);
  mfp_to_flt(update.v,v,3);
  mfp_to_flt(update.q,q,4);
  mfp_to_flt(update.w,w,3);
  mfp_to_flt(update.r,r,12);
  update.yaw = yaw;
  mfp_to_flt(update.weights,weights,12);
  //this is safe, the solver isn't running, and update_problem_data and setup_problem
  //are called from the same thread
  mfp_to_flt(update.traj,state_trajectory,12*problem_configuration.horizon);
  update.alpha = alpha;
  mint_to_u8(update.gait,gait,NUM_CONTACTS*problem_configuration.horizon);

  solve_mpc(&update, &problem_configuration);
  has_solved = 1;
}

void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp) {
  update.max_iterations = max_iter;
  update.rho = rho;
  update.sigma = sigma;
  update.solver_alpha = solver_alpha;
  update.terminate = terminate;
  if(use_jcqp > 1.5)
    update.use_jcqp = 2;
  else if(use_jcqp > 0.5)
    update.use_jcqp = 1;
  else
    update.use_jcqp = 0;
}

void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                float* r, float yaw, float* weights,
                                float* state_trajectory, float alpha, int* gait)
{
  update.alpha = alpha;
  update.yaw = yaw;
  mint_to_u8(update.gait,gait,NUM_CONTACTS*problem_configuration.horizon);
  memcpy((void*)update.p,(void*)p,sizeof(float)*3);
  memcpy((void*)update.v,(void*)v,sizeof(float)*3);
  memcpy((void*)update.q,(void*)q,sizeof(float)*4);
  memcpy((void*)update.w,(void*)w,sizeof(float)*3);
  memcpy((void*)update.r,(void*)r,sizeof(float)*3*NUM_CONTACTS*problem_configuration.horizon);
  memcpy((void*)update.weights,(void*)weights,sizeof(float)*12);
  memcpy((void*)update.traj,(void*)state_trajectory,
          sizeof(float) * 12 * problem_configuration.horizon);
  solve_mpc(&update, &problem_configuration);
  has_solved = 1;
}
void update_problem_data_var_segments(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha,
                                        int* gait, float* dt_segments, float* I_vec, float m)
{
  update.alpha = alpha;
  update.yaw = yaw;
  update.m = m;

  mint_to_u8(update.gait,gait,NUM_CONTACTS*problem_configuration.horizon);
  memcpy((void*)update.p,(void*)p,sizeof(float)*3);
  memcpy((void*)update.v,(void*)v,sizeof(float)*3);
  memcpy((void*)update.q,(void*)q,sizeof(float)*4);
  memcpy((void*)update.w,(void*)w,sizeof(float)*3);
  memcpy((void*)update.r,(void*)r,sizeof(float)*3*NUM_CONTACTS*problem_configuration.horizon);
  memcpy((void*)update.dt_segments,(void*)dt_segments,sizeof(float)*problem_configuration.horizon);
  memcpy((void*)update.weights,(void*)weights,sizeof(float)*12);
  memcpy((void*)update.traj,(void*)state_trajectory,
          sizeof(float) * 12 * problem_configuration.horizon);
  memcpy((void*)update.I_vec,(void*)I_vec,sizeof(float)*3);
  solve_mpc(&update, &problem_configuration);
  has_solved = 1;

}
// void update_problem_data_floats(float* p, float* v, float* q, float* w,
//                                 float* r, float yaw, float* weights,
//                                 float* state_trajectory, float alpha, int* gait, float* dt_segment)
// {
//   update.alpha = alpha;
//   update.yaw = yaw;
//   mint_to_u8(update.gait,gait,NUM_CONTACTS*problem_configuration.horizon);
//   memcpy((void*)update.p,(void*)p,sizeof(float)*3);
//   memcpy((void*)update.v,(void*)v,sizeof(float)*3);
//   memcpy((void*)update.q,(void*)q,sizeof(float)*4);
//   memcpy((void*)update.w,(void*)w,sizeof(float)*3);
//   memcpy((void*)update.r,(void*)r,sizeof(float)*3*NUM_CONTACTS*problem_configuration.horizon);
//   memcpy((void*)update.dt_segment,(void*)dt_segment,sizeof(float)*problem_configuration.horizon);
//   memcpy((void*)update.weights,(void*)weights,sizeof(float)*12);
//   memcpy((void*)update.traj,(void*)state_trajectory,
//           sizeof(float) * 12 * problem_configuration.horizon);
//   solve_mpc(&update, &problem_configuration);
//   has_solved = 1;
// }

void update_x_drag(float x_drag) {
  update.x_drag = x_drag;
}

double get_solution(int index)
{
  if(!has_solved) return 0.f;
  mfp* qs = get_q_soln();
  return qs[index];
}
mfp* get_mpc_solution()
{
  return get_q_soln();
}
// void get_mpc_solution(mfp* &u_star)
// {
//   u_star =  get_q_soln();
// }

// Matrix<fpt,13,13> get_A(){
//   Matrix<fpt,13,13> A = get_A_ct();
//   return A;
// }
//
// Matrix<fpt,13, 3*NUM_CONTACTS> get_B(){
//   Matrix<fpt,13,3*NUM_CONTACTS> B = get_B_ct();
//   return B;
// }
void get_A(Eigen::Matrix<fpt,13,13> &A){
  A = get_A_ct();
}
void get_B(Eigen::Matrix<fpt,13,3*NUM_CONTACTS> &B){
  B = get_B_ct();
}

// Matrix<fpt,13, 3*NUM_CONTACTS> get_B(){
//   Matrix<fpt,13,3*NUM_CONTACTS> B = get_B_ct();
//   return B;
// }

void get_mpc_traj(Eigen::Matrix<fpt,Eigen::Dynamic, 1> & X_mpc, const int horizon){

  get_optimal_mpc_traj(X_mpc, horizon);
}

void get_mpc_Aqpx0(Eigen::Matrix<fpt,Eigen::Dynamic, 1> & Aqp_x0){
     get_Aqpx0(Aqp_x0);
}
void get_mpc_x0(Eigen::Matrix<fpt,13,1> &x0){
      get_x0(x0);
}
void get_mpc_BqpU(Eigen::Matrix<fpt,Eigen::Dynamic, 1> & BqpU, const int horizon){
     get_BqpU(BqpU, horizon);
}
// Eigen::Matrix<fpt,Eigen::Dynamic,13> get_mpc_Aqp(){
//   return get_Aqp();
// }
//
// Eigen::Matrix<fpt,Eigen::Dynamic,Eigen::Dynamic> get_mpc_Bqp(){
//   return get_Bqp();
// }
// Eigen::Matrix<fpt, Eigen::Dynamic, 1> getXref(){
//
//   return getXd();
// }
void getXref(Eigen::Matrix<fpt, Eigen::Dynamic, 1> &Xref){
  Xref = getXd();
}
void get_mpc_Aqp(Eigen::Matrix<fpt, Eigen::Dynamic,13> &Aqp){
  Aqp = get_Aqp();
}
void get_mpc_Bqp(Eigen::Matrix<fpt, Eigen::Dynamic,Eigen::Dynamic> &Bqp){
  Bqp = get_Bqp();
}

void get_mpc_Ub(Eigen::Matrix<fpt,Eigen::Dynamic,1> &Ub){
  Ub = get_Ub();
}
void get_mpc_Lb(Eigen::Matrix<fpt,Eigen::Dynamic,1> &Ul){
  Ul = get_Lb();
}
void get_mpc_C(Eigen::Matrix<fpt,Eigen::Dynamic,Eigen::Dynamic> &C){
  C = get_C();
}
void get_mpc_g(Eigen::Matrix<fpt,Eigen::Dynamic,1> &g){
  g= get_g();
}

void get_mpc_H(Eigen::Matrix<fpt,Eigen::Dynamic,Eigen::Dynamic> &H){

  H = get_H();
}
