#ifndef __GLOBALOPTIONS_H__
#define __GLOBALOPTIONS_H__

#ifdef __cplusplus
extern "C"
{
#endif
/*! QP Header Files */
//#include "qpSWIFT_ldl.h"
#include <math.h>


/*! For easy C++ integration */



/*! QP Macro Functions */
#define MAX(X,Y)  ((X) < (Y) ? (Y) : (X))  /*!< Maximum of two expressions  */
#define MIN(X,Y)  ((X) > (Y) ? (Y) : (X))  /*!< Minimum of two expressions   */


#ifdef MATLAB_MEX_FILE
    #include "mex.h"
    #define MALLOC mxMalloc
    #define FREE mxFree
    #define PRINT mexPrintf
#else
    #include <stdio.h>
    #include "stdlib.h"
    #define MALLOC malloc
    #define FREE free
    #define PRINT printf
#endif

/*! QP Specific Varibles */
#define qp_real double

/*! QP Specific Varibles */
#if (defined _WIN64)
   #define qp_int __int64
#else
   #define qp_int long
#endif


/*! QP SOLVER Settings */
#define MAXIT 100			         /*!< Maximum Number of Iterations */
#define RELTOL 1e-6			      /*!< Residual Error Tolerances */
#define ABSTOL 1e-6			      /*!< s and z Tolerances  */
#define SIGMA 100		         	/*!< Centering Parameter */
#define VERBOSE 0

/*! QP SOLVER FLAGS(EXITCODES) and their Definitions */

#define  QP_OPTIMAL	(0)		/*!< Optimal Solution Found */
#define  QP_KKTFAIL  (1)		/*!< Failure in solving LDL' factorization */
#define  QP_MAXIT	(2)	    	/*!< Maximum Number of Iterations Exceeded */
#define  QP_FATAL	(3)		   /*!< Unknown Problem in Solver */


#ifdef __cplusplus
} // extern "C"
#endif

#endif


/*!
 * @file
 * @authors
 *  - Abhishek Pandala < agp19@vt.edu >
 *  - Yanran Ding < yding35@illinois.edu >
 *  - Hae-won Park < haewonpark@kaist.ac.kr >
 * @version 1.0
 * @date 2019-12-16
 * @copyright GNU Public License
 * @mainpage qpSWIFT
 * @section intro_sec Introduction
 * qpSWIFT is light-weight sparse Quadratic Programming solver targetted for embedded and robotic applications. It employs Primal-Dual Interioir Point method with Mehrotra Predictor
 * corrector step and Nesterov Todd scaling. For solving the linear system of equations, sparse LDL' factorization is used along with approximate minimum degree heuristic to minimize fill-in
 * of the factorizations
 * @section feat_sec Features
 * - Written in ANSI-C
 * - Fully functional Quadratic Programming solver for embedded applications
 * - Code Generation for target platform
 * - Tested on multiple target architectures
 *    + x86
 *    + x86_64
 *    + ARM
 *    + PowerPC
 * - Support for multiple interfaces
 *    + C/C++
 *    + Matlab
 *    + Simulink
 *
 *
 * @section case_study Case Studies
 * - BAMBY
 * - Ghost Robotics Vision60
 *
 *
 * @section updates Future Updates
 *  - Quadratic Program with only equality constraints
 *  - Support for Python, R and Julia
 *
 *
 * @section note Note
 *  The project is still in active development. Feedback is highly appreciated. For any queries, please write to haewonpark@kaist.ac.kr
 *
 *
 * @section cite Citing qpSWIFT
 * If you like qpSWIFT and are using it in your work, please cite the following paper \n
 *
 * @code{.html}
 * @article{pandala2019qpswift,
 * title     = {qpSWIFT: A Real-Time Sparse Quadratic Program Solver for Robotic Applications},
 * author    = {Pandala, Abhishek Goud and Ding, Yanran and Park, Hae-Won},
 * journal   = {IEEE Robotics and Automation Letters},
 * volume    = {4},
 * number    = {4},
 * pages     = {3355--3362},
 * year      = {2019},
 * publisher = {IEEE}
 * }
 * @endcode
*/
