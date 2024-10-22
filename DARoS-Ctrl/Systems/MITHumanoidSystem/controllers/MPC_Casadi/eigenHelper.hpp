#pragma once
#include <eigen3/Eigen/Core>
#include <casadi/casadi.hpp>
#include <vector>
#include </usr/local/include/eigen3/Eigen/QR>
#include </usr/local/include/eigen3/unsupported/Eigen/KroneckerProduct>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace casadi;

Eigen::MatrixXf dmToEigenf(casadi::DM m);
Eigen::MatrixXd dmToEigen(casadi::DM m);
Eigen::VectorXd dmToEigenVector (casadi::DM m);
Eigen::VectorXf dmToEigenVectorf (casadi::DM m);

Eigen::MatrixXd arrayToEigen(double m[], int row , int col);
casadi::DM EigenTodm(Eigen::MatrixXd matrix);
casadi::DM EigenMatrixTodm(Eigen::MatrixXd matrix);
casadi::DM EigenMatrixfTodm(Eigen::MatrixXf matrix);
casadi::DM EigenVectorTodm(Eigen::VectorXd vec);
casadi::DM EigenVectorfTodm(Eigen::VectorXf vec);

Eigen::Matrix3d yprToRotmat(double roll, double pitch, double yaw);

Eigen::Matrix3d skew(Eigen::Vector3d v);

Eigen::MatrixXd get_N();

Eigen::MatrixXd get_D(Eigen::Vector3d v);

Eigen::MatrixXd get_F(Eigen::Vector3d v);

Eigen::VectorXd vectorize(Eigen::MatrixXd m);

Eigen::Matrix3d euler2Rot( const double roll, const double pitch, const double yaw );

casadi::DM cross2d(casadi::DM x, casadi::DM y);

struct Coeffs {
  Eigen::Matrix3d CE_eta;
  Eigen::Matrix3d CE_w;
  Eigen::Vector3d CE_c;
};

Coeffs eta_co_R(Eigen::Matrix3d Rop, Eigen::Vector3d wop, double dt);

struct Woeffs {
  Eigen::Matrix3d Cw_x;
  Eigen::Matrix3d Cw_eta;
  Eigen::Matrix3d Cw_w;
  Eigen::MatrixXd Cw_u;
  Eigen::Vector3d Cw_c;
};

Woeffs eta_co_w(Eigen::Vector3d Xop, Eigen::Matrix3d R_op, Eigen::Vector3d w_op, 
              Eigen::VectorXd f_op, double dt, Eigen::MatrixXd pf, Eigen::Matrix3d J, Eigen::Matrix3d Jinv);


// casadi::Function conv_3by3();

casadi::Function expm_casadi(double dt);