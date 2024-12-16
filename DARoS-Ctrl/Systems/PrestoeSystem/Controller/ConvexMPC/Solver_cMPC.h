#ifndef Biped_solver_mpc
#define Biped_solver_mpc


#include <eigen3/Eigen/Dense>
#include "cMPC_types.h"
#include "cMPC_BipedInterface.h"
#include <iostream>
#include <stdio.h>


using Eigen::Matrix;
using Eigen::Quaternionf;
using Eigen::Quaterniond;

template <class T>
void print_array(T* array, u16 rows, u16 cols)
{
    for(u16 r = 0; r < rows; r++)
    {
        for(u16 c = 0; c < cols; c++)
            std::cout<<(fpt)array[c+r*cols]<<" ";
        printf("\n");
    }
}

template <class T>
void print_named_array(const char* name, T* array, u16 rows, u16 cols)
{
    printf("%s:\n",name);
    print_array(array,rows,cols);
}

//print named variable
template <class T>
void pnv(const char* name, T v)
{
    printf("%s: ",name);
    std::cout<<v<<std::endl;
}

template <class T>
T t_min(T a, T b)
{
    if(a<b) return a;
    return b;
}

template <class T>
T sq(T a)
{
    return a*a;
}


void solve_mpc(update_data_t* update, problem_setup* setup);

void quat_to_rpy(Quaternionf q, Matrix<fpt,3,1>& rpy);
void ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,NUM_CONTACTS> r_feet, Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13, 3*NUM_CONTACTS>& B, float x_drag);
void resize_qp_mats(s16 horizon);
void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,3*NUM_CONTACTS> Bc,fpt dt,s16 horizon);
void updateQPMats(fpt m,
                  Matrix<fpt,3,3> R_yaw,
                  Matrix<fpt,3,NUM_CONTACTS> * r_feet,
                  fpt* dt_segments,
                  s16 horizon,
                  float x_drag);
mfp* get_q_soln();

Matrix<fpt,13,13> get_A_ct();
Matrix<fpt,13,3*NUM_CONTACTS> get_B_ct();
void get_optimal_mpc_traj(Matrix<fpt,Eigen::Dynamic, 1> & X_mpc, const int horizon);
Matrix<fpt, Eigen::Dynamic, 1> getXd();
void get_Aqpx0(Matrix<fpt,Eigen::Dynamic, 1> & Aqp_x0);
void get_x0(Matrix<fpt,13,1> &x0);
void get_BqpU(Matrix<fpt,Eigen::Dynamic, 1> & BqpU, const int horizon);
Matrix<fpt,Eigen::Dynamic,13> get_Aqp();
Matrix<fpt,Eigen::Dynamic,Eigen::Dynamic> get_Bqp();

Matrix<fpt,Eigen::Dynamic,1> get_Ub();
Matrix<fpt,Eigen::Dynamic,1> get_Lb();
Matrix<fpt,Eigen::Dynamic,Eigen::Dynamic> get_C();
Matrix<fpt,Eigen::Dynamic,Eigen::Dynamic> get_H();
Matrix<fpt,Eigen::Dynamic,1> get_g();

#endif
