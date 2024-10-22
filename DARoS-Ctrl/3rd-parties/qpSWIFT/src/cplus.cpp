#include "Prime.h"
#include "iostream"
#include "Eigen/Dense"


int main(int argv, char **argc){

    Eigen::Matrix<double, 3, 3> P;
    Eigen::Matrix<double, 3, 3> G;
    Eigen::Matrix<double, 3, 1> q;
    Eigen::Matrix<double, 3, 1> h;
    Eigen::Matrix<double, 1, 3> A;
    Eigen::Matrix<double, 1, 1> b;

    P << 65.0, -22, -16,
         -22.0, 14, 7,
         -16, 7, 5;
      
    q << 3.0, 2.0, 3.0;

    G << 1.0, 2.0, 1.0,
        2.0, 0.0, 1.0,
        -1.0, 2.0, -1.0;

    h << 3.0, 2.0, -2.0;

    A << 1.0, 1.0, 1.0;

    b << 1.0;

    QP* myQP;

	qp_int n = 3;	/* Number of Decision Variables */
	qp_int m = 3;	/* Number of Inequality Constraints */
	qp_int p = 1;   /* Number of equality Constraints */

	// Input Matrices in Column Major Format; Refer to GlobalOptions.h to see what qp_int and qp_real are //
	// qp_real P_dense[9] = { 65,-22,-16,-22,14,7,-16,7,5 };          /* P matrix in Column Major Format */
	// qp_real A_dense[3] = { 1.,1.,1. };                             /* A matrix in Column Major Format */
	// qp_real G_dense[9] = { 1., 2., -1.,2., 0., 2.,1., 1., -1. };   /* G matrix in Column Major Format */

	// qp_real h_dense[3] = { 3., 2., -2. };						   /* h vector */
	// qp_real c_dense[3] = { -13.0,15.0,7.0 };					   /* c vector */
	// qp_real b_dense[1] = { 1.0 };								   /* b vector */


  

	// Setup Function //
	myQP = QP_SETUP_dense(n, m, p, P.data(), A.data(), G.data(), q.data(), h.data(), b.data(), NULL);

	/****************************************
	*	After this, you can change the solver settings like this
	*	myQP->options->maxit  = 30   (to change the maximum number of iterations to 30; default is 100)
	*	myQP->options->reltol = 1e-3 (to change the Relative tolerance to 1e-3; default is 1e-6)
	*	myQP->options->abstol  = 1e-3 (to change the Absolute tolerance to 1e-3; default is 1e-6)
	*	myQP->options->SIGMA  = 50 (to change the SIGMA to 50; default is 100; recommended not to change this)
	*	myQP->options->VERBOSE  = 0 (displays no output when set to 0; default is 1 which corresponds to complete verbose mode)
	*
	*
	*
	******************************************/

	/* The Solution can be found as real pointer in myQP->x;It is an array of Dimension n*/

	qp_int ExitCode = QP_SOLVE(myQP);

	std::cout << "Exit Code is " << ExitCode << std::endl;

	std::cout << "Solution" << std::endl;

	for (int i = 0; i < 3;++i)
		std::cout << "x[" << i << "]: " << myQP->x[i] << std::endl;

	QP_CLEANUP_dense(myQP);



    return 0;
}
