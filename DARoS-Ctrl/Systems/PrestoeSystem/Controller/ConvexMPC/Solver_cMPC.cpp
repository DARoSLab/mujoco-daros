#include "Solver_cMPC.h"
#include "BipedRobotState.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>
#include <stdio.h>
#include <sys/time.h>
#include <utilities/Timer.h>

// #define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10


//big enough to act like infinity, small enough to avoid numerical weirdness.

BipedRobotState rs;
using std::cout;
using std::endl;
using Eigen::Dynamic;

//qpOASES::real_t a;
Matrix<fpt,Dynamic,Dynamic> B_qp_trans_S;

Matrix<fpt,Dynamic,13> A_qp;
Matrix<fpt,Dynamic,Dynamic> B_qp;
Matrix<fpt,13,3*NUM_CONTACTS> Bdt;
Matrix<fpt,13,13> Adt;
Matrix<fpt,13 + 3*NUM_CONTACTS, 13 + 3*NUM_CONTACTS> ABc,expmm;
Matrix<fpt,Dynamic,Dynamic> S;
Matrix<fpt,Dynamic,1> X_d;
Matrix<fpt,Dynamic,1> U_b;
Matrix<fpt,Dynamic,Dynamic> fmat;

Matrix<fpt,Dynamic,Dynamic> qH;
Matrix<fpt,Dynamic,1> qg;

Matrix<fpt,Dynamic,Dynamic> eye_h;

qpOASES::real_t* H_qpoases;
qpOASES::real_t* g_qpoases;
qpOASES::real_t* A_qpoases;
qpOASES::real_t* lb_qpoases;
qpOASES::real_t* ub_qpoases;
qpOASES::real_t* q_soln;

qpOASES::real_t* H_red;
qpOASES::real_t* g_red;
qpOASES::real_t* A_red;
qpOASES::real_t* lb_red;
qpOASES::real_t* ub_red;
qpOASES::real_t* q_red;
u8 real_allocated = 0;


char var_elim[2000];
char con_elim[2000];




mfp* get_q_soln()
{
  return q_soln;
}

s8 near_zero(fpt a)
{
  return (a < 0.01 && a > -.01) ;
}

s8 near_one(fpt a)
{
  return near_zero(a-1);
}
void matrix_to_real(qpOASES::real_t* dst, Matrix<fpt,Dynamic,Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for(s16 r = 0; r < rows; r++)
  {
    for(s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r,c);
      a++;
    }
  }
}


void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,3*NUM_CONTACTS> Bc,fpt dt,s16 horizon)
{
  ABc.setZero();
  ABc.block(0,0,13,13) = Ac;
  ABc.block(0,13,13,3*NUM_CONTACTS) = Bc;
  ABc = dt*ABc;
  expmm = ABc.exp();
  Adt = expmm.block(0,0,13,13);
  Bdt = expmm.block(0,13,13,3*NUM_CONTACTS);
#ifdef K_PRINT_EVERYTHING
  cout<<"Adt: \n"<<Adt<<"\nBdt:\n"<<Bdt<<endl;
#endif
  if(horizon > 39) {
    throw std::runtime_error("horizon is too long!");
  }

  Matrix<fpt,13,13> powerMats[40];
  powerMats[0].setIdentity();
  for(int i = 1; i < horizon+1; i++) {
    powerMats[i] = Adt * powerMats[i-1];
  }

  for(s16 r = 0; r < horizon; r++)
  {
    A_qp.block(13*r,0,13,13) = powerMats[r+1];//Adt.pow(r+1);
    for(s16 c = 0; c < horizon; c++)
    {
      if(r >= c)
      {
        s16 a_num = r-c;
        B_qp.block(13*r,3*NUM_CONTACTS*c,13,3*NUM_CONTACTS) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
      }
    }
  }

#ifdef K_PRINT_EVERYTHING
  cout<<"AQP:\n"<<A_qp<<"\nBQP:\n"<<B_qp<<endl;
#endif
}

void resize_qp_mats(s16 horizon)
{
  int mcount = 0;
  int h2 = horizon*horizon;

  A_qp.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon*1;

  B_qp.resize(13*horizon, 3*NUM_CONTACTS*horizon);
  mcount += 13*h2*3*NUM_CONTACTS;

  S.resize(13*horizon, 13*horizon);
  mcount += 13*13*h2;

  X_d.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;

  U_b.resize(5*NUM_CONTACTS*horizon, Eigen::NoChange);
  mcount += 5*NUM_CONTACTS*horizon;

  fmat.resize(5*NUM_CONTACTS*horizon, 3*NUM_CONTACTS*horizon);
  mcount += 5*NUM_CONTACTS*3*NUM_CONTACTS*h2;

  qH.resize(3*NUM_CONTACTS*horizon, 3*NUM_CONTACTS*horizon);
  mcount += 3*NUM_CONTACTS*3*NUM_CONTACTS*h2;

  qg.resize(3*NUM_CONTACTS*horizon, Eigen::NoChange);
  mcount += 3*NUM_CONTACTS*horizon;

  eye_h.resize(3*NUM_CONTACTS*horizon, 3*NUM_CONTACTS*horizon);
  mcount += 3*NUM_CONTACTS*3*NUM_CONTACTS*horizon;

  //printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  fmat.setZero();
  qH.setZero();
  eye_h.setIdentity();

  //TODO: use realloc instead of free/malloc on size changes

  if(real_allocated)
  {

    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  H_qpoases = (qpOASES::real_t*)malloc(3*NUM_CONTACTS*3*NUM_CONTACTS*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 3*NUM_CONTACTS*3*NUM_CONTACTS*h2;
  g_qpoases = (qpOASES::real_t*)malloc(3*NUM_CONTACTS*1*horizon*sizeof(qpOASES::real_t));
  mcount += 3*NUM_CONTACTS*horizon;
  A_qpoases = (qpOASES::real_t*)malloc(3*NUM_CONTACTS*5*NUM_CONTACTS*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 3*NUM_CONTACTS*5*NUM_CONTACTS*h2;
  lb_qpoases = (qpOASES::real_t*)malloc(5*NUM_CONTACTS*1*horizon*sizeof(qpOASES::real_t));
  mcount += 5*NUM_CONTACTS*horizon;
  ub_qpoases = (qpOASES::real_t*)malloc(5*NUM_CONTACTS*1*horizon*sizeof(qpOASES::real_t));
  mcount += 5*NUM_CONTACTS*horizon;
  q_soln = (qpOASES::real_t*)malloc(3*NUM_CONTACTS*horizon*sizeof(qpOASES::real_t));
  mcount += 3*NUM_CONTACTS*horizon;

  H_red = (qpOASES::real_t*)malloc(3*NUM_CONTACTS*3*NUM_CONTACTS*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 3*NUM_CONTACTS*3*NUM_CONTACTS*h2;
  g_red = (qpOASES::real_t*)malloc(3*NUM_CONTACTS*1*horizon*sizeof(qpOASES::real_t));
  mcount += 3*NUM_CONTACTS*horizon;
  A_red = (qpOASES::real_t*)malloc(3*NUM_CONTACTS*5*NUM_CONTACTS*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 3*NUM_CONTACTS*5*NUM_CONTACTS*h2;
  lb_red = (qpOASES::real_t*)malloc(5*NUM_CONTACTS*1*horizon*sizeof(qpOASES::real_t));
  mcount += 5*NUM_CONTACTS*horizon;
  ub_red = (qpOASES::real_t*)malloc(5*NUM_CONTACTS*1*horizon*sizeof(qpOASES::real_t));
  mcount += 5*NUM_CONTACTS*horizon;
  q_red = (qpOASES::real_t*)malloc(3*NUM_CONTACTS*horizon*sizeof(qpOASES::real_t));
  mcount += 3*NUM_CONTACTS*horizon;
  real_allocated = 1;

  //printf("malloc'd %d floating point numbers.\n",mcount);



#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n",horizon);
#endif
}

inline Matrix<fpt,3,3> cross_mat(Matrix<fpt,3,3> I_inv, Matrix<fpt,3,1> r)
{
  Matrix<fpt,3,3> cm;
  cm << 0.f, -r(2), r(1),
     r(2), 0.f, -r(0),
     -r(1), r(0), 0.f;
  return I_inv * cm;
}
//continuous time state space matrices.
void ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,NUM_CONTACTS> r_feet, Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13,3*NUM_CONTACTS>& B, float x_drag)
{
  A.setZero();
  A(3,9) = 1.f;
  A(11,9) = x_drag;
  A(4,10) = 1.f;
  A(5,11) = 1.f;

  A(11,12) = 1.f;
  A(12,12) = 0.f; // Daniel for gravity
  A.block(0,6,3,3) = R_yaw.transpose();

  B.setZero();
  Matrix<fpt,3,3> I_inv = I_world.inverse();

  for(s16 b = 0; b < NUM_CONTACTS; b++)
  {
    B.block(6,b*3,3,3) = cross_mat(I_inv,r_feet.col(b));
    B.block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
  }

}


void quat_to_rpy(Quaternionf q, Matrix<fpt,3,1>& rpy)
{
  //from my MATLAB implementation

  //edge case!
  fpt as = t_min(-2.*(q.x()*q.z()-q.w()*q.y()),.99999);
  rpy(0) = atan2(2.f*(q.x()*q.y()+q.w()*q.z()),sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f*(q.y()*q.z()+q.w()*q.x()),sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));

}
void print_problem_setup(problem_setup* setup)
{
  printf("DT: %.3f\n",setup->dt);
  printf("Mu: %.3f\n",setup->mu);
  printf("F_Max: %.3f\n",setup->f_max);
  printf("Horizon: %d\n",setup->horizon);
}

void print_update_data(update_data_t* update, s16 horizon)
{
  print_named_array("p",update->p,1,3);
  print_named_array("v",update->v,1,3);
  print_named_array("q",update->q,1,4);
  print_named_array("w",update->r,3,4);
  pnv("Yaw",update->yaw);
  print_named_array("weights",update->weights,1,12);
  print_named_array("trajectory",update->traj,horizon,12);
  pnv("Alpha",update->alpha);
  print_named_array("gait",update->gait,horizon,NUM_CONTACTS);
}


Matrix<fpt,13,1> x_0;
Matrix<fpt,3,3> I_world;
Matrix<fpt,13,13> A_ct;
Matrix<fpt,13,3*NUM_CONTACTS> B_ct_r;

Matrix<fpt,13,13> get_A_ct(){
  return A_ct;
}
Matrix<fpt,13,3*NUM_CONTACTS> get_B_ct(){
  return B_ct_r;
}

void updateQPMats(fpt m,
                  Matrix<fpt,3,3> R_yaw,
                  Matrix<fpt,3, NUM_CONTACTS> * r_feet,
                  fpt* dt_segments,
                  s16 horizon,
                  float x_drag)
{
  // Matrix<fpt,13,13> A_ct;
  // Matrix<fpt,13,3*NUM_CONTACTS> B_ct_r;
  Matrix<fpt,13,3*NUM_CONTACTS> Bdts[horizon];
  Matrix<fpt,13,13> powerMats[40];
  powerMats[0].setIdentity();

  for(int k(1); k<horizon+1; ++k){
    ct_ss_mats(I_world, m, r_feet[k-1], R_yaw, A_ct, B_ct_r, x_drag);
    // std::cout << "Act: " << A_ct << '\n';
    // std::cout << "Bct: " << B_ct_r << '\n';
    // std::cout << "dt: " << dt_segments[k] << '\n';
    ABc.setZero();
    ABc.block(0,0,13,13) = A_ct;
    ABc.block(0,13,13,3*NUM_CONTACTS) = B_ct_r;
    ABc = dt_segments[k]*ABc;
    expmm = ABc.exp();
    Adt = expmm.block(0,0,13,13);
    Bdt = expmm.block(0,13,13,3*NUM_CONTACTS);

    powerMats[k] = Adt * powerMats[k-1];
    Bdts[k-1] = Bdt;
  }

  for(s16 r = 0; r < horizon; r++)
  {
    A_qp.block(13*r,0,13,13) = powerMats[r+1];//Adt.pow(r+1);
    for(s16 c = 0; c < horizon; c++)
    {
      if(r >= c)
      {
        s16 a_num = r-c;
        B_qp.block(13*r,3*NUM_CONTACTS*c,13,3*NUM_CONTACTS) = powerMats[a_num]* Bdts[c];
      }
    }
  }
  //For debugging i.e next state
  ct_ss_mats(I_world, m, r_feet[0], R_yaw, A_ct, B_ct_r, x_drag);
  #ifdef K_PRINT_EVERYTHING
    cout<<"AQP:\n"<<A_qp<<"\nBQP:\n"<<B_qp<<endl;
  #endif
}

void solve_mpc(update_data_t* update, problem_setup* setup)
{

  if(setup->general_form)
    rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw, update->I_vec, update->m, setup->horizon);
  else
    rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw);
#ifdef K_PRINT_EVERYTHING

  printf("-----------------\n");
  printf("   PROBLEM DATA  \n");
  printf("-----------------\n");
  print_problem_setup(setup);

  printf("-----------------\n");
  printf("    ROBOT DATA   \n");
  printf("-----------------\n");
  rs.print();
  print_update_data(update,setup->horizon);
#endif

  //roll pitch yaw
  Matrix<fpt,3,1> rpy;
  quat_to_rpy(rs.q,rpy);

  //initial state (13 state representation)
  x_0 << rpy(2), rpy(1), rpy(0), rs.p , rs.w, rs.v, -9.8f;
  I_world = rs.R_yaw * rs.I_body * rs.R_yaw.transpose(); //original
  //I_world = rs.R_yaw.transpose() * rs.I_body * rs.R_yaw;
  //cout<<rs.R_yaw<<endl;


  if(setup->general_form){

    updateQPMats(rs.m,
                 rs.R_yaw,
                 rs.r_feet_full,
                 update->dt_segments,
                 setup->horizon,
                 update->x_drag);

  }else{
    ct_ss_mats(I_world,rs.m,rs.r_feet,rs.R_yaw,A_ct,B_ct_r, update->x_drag);
#ifdef K_PRINT_EVERYTHING
  cout<<"Initial state: \n"<<x_0<<endl;
  cout<<"World Inertia: \n"<<I_world<<endl;
  cout<<"A CT: \n"<<A_ct<<endl;
  cout<<"B CT (simplified): \n"<<B_ct_r<<endl;
#endif
    //QP matrices
    c2qp(A_ct,B_ct_r,setup->dt,setup->horizon);
  }
  //weights
  Matrix<fpt,13,1> full_weight;
  for(u8 i = 0; i < 12; i++)
    full_weight(i) = update->weights[i];
  full_weight(12) = 0.f;
  S.diagonal() = full_weight.replicate(setup->horizon,1);
  B_qp_trans_S = B_qp.transpose()*S;

  //trajectory
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < 12; j++)
      X_d(13*i+j,0) = update->traj[12*i+j];
  }
  //cout<<"XD:\n"<<X_d<<endl;



  //note - I'm not doing the shifting here.
  s16 k = 0;
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < NUM_CONTACTS; j++)
    {
      U_b(5*k + 0) = BIG_NUMBER;
      U_b(5*k + 1) = BIG_NUMBER;
      U_b(5*k + 2) = BIG_NUMBER;
      U_b(5*k + 3) = BIG_NUMBER;
      U_b(5*k + 4) = update->gait[i*NUM_CONTACTS + j] * setup->f_max;
      k++;
    }
  }

  fpt mu = 1.f/setup->mu;
  Matrix<fpt,5,3> f_block;

  f_block <<  mu, 0,  1.f,
          -mu, 0,  1.f,
          0,  mu, 1.f,
          0, -mu, 1.f,
          0,   0, 1.f;

  for(s16 i = 0; i < setup->horizon*NUM_CONTACTS; i++)
  {
    fmat.block(i*5,i*3,5,3) = f_block;
  }




  //qH = 2*(B_qp.transpose()*S*B_qp + update->alpha*eye_h);
  //qg = 2*B_qp.transpose()*S*(A_qp*x_0 - X_d);

  qH = 2*(B_qp_trans_S*B_qp + update->alpha*eye_h);
  qg = 2*B_qp_trans_S*(A_qp*x_0 - X_d);



  matrix_to_real(H_qpoases,qH,setup->horizon*3*NUM_CONTACTS, setup->horizon*3*NUM_CONTACTS);
  matrix_to_real(g_qpoases,qg,setup->horizon*3*NUM_CONTACTS, 1);
  matrix_to_real(A_qpoases,fmat,setup->horizon*5*NUM_CONTACTS, setup->horizon*3*NUM_CONTACTS);
  matrix_to_real(ub_qpoases,U_b,setup->horizon*5*NUM_CONTACTS, 1);

  for(s16 i = 0; i < 5*NUM_CONTACTS*setup->horizon; i++)
    lb_qpoases[i] = 0.0f;

  s16 num_constraints = 5*NUM_CONTACTS*setup->horizon;
  s16 num_variables = 3*NUM_CONTACTS*setup->horizon;




  int new_vars = num_variables;
  int new_cons = num_constraints;

  for(int i =0; i < num_constraints; i++)
    con_elim[i] = 0;

  for(int i = 0; i < num_variables; i++)
    var_elim[i] = 0;


  for(int i = 0; i < num_constraints; i++)
  {
    if(! (near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i]))) continue;
    double* c_row = &A_qpoases[i*num_variables];
    for(int j = 0; j < num_variables; j++)
    {
      if(near_one(c_row[j]))
      {
        new_vars -= 3;
        new_cons -= 5;
        int cs = (j*5)/3 -3;
        var_elim[j-2] = 1;
        var_elim[j-1] = 1;
        var_elim[j  ] = 1;
        con_elim[cs] = 1;
        con_elim[cs+1] = 1;
        con_elim[cs+2] = 1;
        con_elim[cs+3] = 1;
        con_elim[cs+4] = 1;
      }
    }
  }
  //if(new_vars != num_variables)
  if(1==1)
  {
    int var_ind[new_vars];
    int con_ind[new_cons];
    int vc = 0;
    for(int i = 0; i < num_variables; i++)
    {
      if(!var_elim[i])
      {
        if(!(vc<new_vars))
        {
          printf("BAD ERROR 1\n");
        }
        var_ind[vc] = i;
        vc++;
      }
    }
    vc = 0;
    for(int i = 0; i < num_constraints; i++)
    {
      if(!con_elim[i])
      {
        if(!(vc<new_cons))
        {
          printf("BAD ERROR 1\n");
        }
        con_ind[vc] = i;
        vc++;
      }
    }
    for(int i = 0; i < new_vars; i++)
    {
      int olda = var_ind[i];
      g_red[i] = g_qpoases[olda];
      for(int j = 0; j < new_vars; j++)
      {
        int oldb = var_ind[j];
        H_red[i*new_vars + j] = H_qpoases[olda*num_variables + oldb];
      }
    }

    for (int con = 0; con < new_cons; con++)
    {
      for(int st = 0; st < new_vars; st++)
      {
        float cval = A_qpoases[(num_variables*con_ind[con]) + var_ind[st] ];
        A_red[con*new_vars + st] = cval;
      }
    }
    for(int i = 0; i < new_cons; i++)
    {
      int old = con_ind[i];
      ub_red[i] = ub_qpoases[old];
      lb_red[i] = lb_qpoases[old];
    }

    Timer solve_timer;
    qpOASES::QProblem problem_red (new_vars, new_cons);
    qpOASES::Options op;
    qpOASES::int_t nWSR = 100;
    op.setToMPC();
    // op.setToReliable();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);
    // int_t nWSR = 50000;
    // printf("num vars before: %d after: %d \n", num_variables, new_vars);
    // printf("num cons before: %d after: %d \n", num_constraints, new_cons);

    int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
    // (void)rval;
    if(rval != qpOASES::SUCCESSFUL_RETURN)
      printf("failed to Init!\n");
    int rval2 = problem_red.getPrimalSolution(q_red);
    if(rval2 != qpOASES::SUCCESSFUL_RETURN)
      printf("failed to solve!\n");
    // auto opt_cost = problem_red.getObjVal();
    // printf("opt cost: %f\n", opt_cost);
    // printf("num active constraints: %d \n", problem_red.getNAC());
    // printf("num in active constraints: %d \n", problem_red.getNIAC());

    // printf("solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);


    vc = 0;
    for(int i = 0; i < num_variables; i++)
    {
      if(var_elim[i])
      {
        q_soln[i] = 0.0f;
      }
      else
      {
        q_soln[i] = q_red[vc];
        vc++;
      }
    }
  }

#ifdef K_PRINT_EVERYTHING
//cout<<"fmat:\n"<<fmat<<endl;
#endif



}


void get_optimal_mpc_traj(Matrix<fpt,Dynamic, 1> & X_mpc, const int horizon){

  Matrix<fpt, 3*NUM_CONTACTS*K_MAX_GAIT_SEGMENTS, 1> U_star;
  for(s16 k=0; k<3*NUM_CONTACTS*horizon; ++k){

    U_star[k] = q_soln[k];
  }
  X_mpc = A_qp*x_0;
  X_mpc += B_qp*U_star.head(3*NUM_CONTACTS*horizon);

}
void get_Aqpx0(Matrix<fpt,Dynamic, 1> & Aqp_x0){
  Aqp_x0 = A_qp*x_0;
}
void get_x0(Matrix<fpt,13,1> &x0){
  x0 = x_0;
}
void get_BqpU(Matrix<fpt,Dynamic, 1> & BqpU, const int horizon){
  Matrix<fpt, 3*NUM_CONTACTS*K_MAX_GAIT_SEGMENTS, 1> U_star;
  for(s16 k=0; k<3*NUM_CONTACTS*horizon; ++k){
    U_star[k] = q_soln[k];
  }
  BqpU= B_qp*U_star.head(3*NUM_CONTACTS*horizon);
}
Matrix<fpt,Dynamic,13> get_Aqp(){
  return A_qp;
}

Matrix<fpt,Dynamic,Dynamic> get_Bqp(){
  return B_qp;
}


Matrix<fpt, Dynamic, 1> getXd(){

  return X_d;
}

Matrix<fpt,Dynamic,1> get_Ub(){return U_b;}
Matrix<fpt,Dynamic,1> get_Lb(){return 0.* U_b;}
Matrix<fpt,Dynamic,Dynamic> get_C(){return fmat;}
Matrix<fpt,Dynamic,Dynamic> get_H(){return qH;}
Matrix<fpt,Dynamic,1> get_g(){ return qg;}
