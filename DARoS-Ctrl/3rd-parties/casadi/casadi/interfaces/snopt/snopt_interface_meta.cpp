/*
 *    This file is part of CasADi.
 *
 *    CasADi -- A symbolic framework for dynamic optimization.
 *    Copyright (C) 2010-2023 Joel Andersson, Joris Gillis, Moritz Diehl,
 *                            KU Leuven. All rights reserved.
 *    Copyright (C) 2011-2014 Greg Horn
 *
 *    CasADi is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    CasADi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with CasADi; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


      #include "snopt_interface.hpp"
      #include <string>

      const std::string casadi::SnoptInterface::meta_doc=
      "\n"
"SNOPT interface\n"
"\n"
"\n"
">List of available options\n"
"\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"|       Id        |      Type       |     Default     |   Description   |\n"
"+=================+=================+=================+=================+\n"
"| Backup basis    | OT_INT      | None            | 0 * output      |\n"
"| file            |                 |                 | extra basis map |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Central         | OT_DOUBLE         | None            | 6.7e-5 *        |\n"
"| difference      |                 |                 | (Function       |\n"
"| interval        |                 |                 | precision)^1/3  |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Check frequency | OT_INT      | None            | 60 * test row   |\n"
"|                 |                 |                 | residuals kAx - |\n"
"|                 |                 |                 | sk              |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Crash option    | OT_INT      | None            | 3 * first basis |\n"
"|                 |                 |                 | is essentially  |\n"
"|                 |                 |                 | triangular      |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Crash tolerance | OT_DOUBLE         | None            | 0.100           |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Debug level     | OT_INT      | None            | 0 * for         |\n"
"|                 |                 |                 | developers      |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Derivative      | OT_INT      | None            | 3               |\n"
"| level           |                 |                 |                 |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Difference      | OT_DOUBLE         | None            | 5.5e-7 *        |\n"
"| interval        |                 |                 | (Function       |\n"
"|                 |                 |                 | precision)^1/2  |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Dump file       | OT_INT      | None            | 0 * output Load |\n"
"|                 |                 |                 | data            |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Elastic weight  | OT_DOUBLE         | None            | 1.0e+4 * used   |\n"
"|                 |                 |                 | only during     |\n"
"|                 |                 |                 | elastic mode    |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Expand          | OT_INT      | None            | 10000 * for     |\n"
"| frequency       |                 |                 | anti-cycling    |\n"
"|                 |                 |                 | procedure       |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Factorization   | OT_INT      | None            | 50 * 100 for    |\n"
"| frequency       |                 |                 | LPs             |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Function        | OT_DOUBLE         | None            | 3.0e-13 * e^0.8 |\n"
"| precision       |                 |                 | (almost full    |\n"
"|                 |                 |                 | accuracy)       |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Hessian         | OT_STRING       | None            | full memory *   |\n"
"|                 |                 |                 | default if n1   |\n"
"|                 |                 |                 | 75  limited     |\n"
"|                 |                 |                 | memory *        |\n"
"|                 |                 |                 | default if n1 > |\n"
"|                 |                 |                 | 75              |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Hessian flush   | OT_INT      | None            | 999999 * no     |\n"
"|                 |                 |                 | flushing        |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Hessian         | OT_INT      | None            | 999999 * for    |\n"
"| frequency       |                 |                 | full Hessian    |\n"
"|                 |                 |                 | (never reset)   |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Hessian updates | OT_INT      | None            | 10 * for        |\n"
"|                 |                 |                 | limited memory  |\n"
"|                 |                 |                 | Hessian         |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Insert file     | OT_INT      | None            | 0 * input in    |\n"
"|                 |                 |                 | industry format |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Iterations      | OT_INT      | None            | 10000 * or 20m  |\n"
"| limit           |                 |                 | if that is more |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| LU              | OT_STRING       | None            | LU partial      |\n"
"|                 |                 |                 | pivoting *      |\n"
"|                 |                 |                 | default         |\n"
"|                 |                 |                 | threshold       |\n"
"|                 |                 |                 | pivoting        |\n"
"|                 |                 |                 | strategy  LU    |\n"
"|                 |                 |                 | rook pivoting * |\n"
"|                 |                 |                 | threshold rook  |\n"
"|                 |                 |                 | pivoting  LU    |\n"
"|                 |                 |                 | complete        |\n"
"|                 |                 |                 | pivoting *      |\n"
"|                 |                 |                 | threshold       |\n"
"|                 |                 |                 | complete        |\n"
"|                 |                 |                 | pivoting        |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| LU factor       | OT_DOUBLE         | None            | 3.99 * for NP   |\n"
"| tolerance       |                 |                 | (100.0 for LP)  |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| LU singularity  | OT_DOUBLE         | None            | 0.000           |\n"
"| tolerance       |                 |                 |                 |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| LU update       | OT_DOUBLE         | None            | 3.99 * for NP ( |\n"
"| tolerance       |                 |                 | 10.0 for LP)    |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Linesearch      | OT_DOUBLE         | None            | 0.9 * smaller   |\n"
"| tolerance       |                 |                 | for more        |\n"
"|                 |                 |                 | accurate search |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Load file       | OT_INT      | None            | 0 * input names |\n"
"|                 |                 |                 | and values      |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Major           | OT_DOUBLE         | None            | 1.0e-6 * target |\n"
"| feasibility     |                 |                 | nonlinear       |\n"
"| tolerance       |                 |                 | constraint      |\n"
"|                 |                 |                 | violation       |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Major           | OT_INT      | None            | 1000 * or m if  |\n"
"| iterations      |                 |                 | that is more    |\n"
"| limit           |                 |                 |                 |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Major           | OT_DOUBLE         | None            | 1.0e-6 * target |\n"
"| optimality      |                 |                 | complementarity |\n"
"| tolerance       |                 |                 | gap             |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Major print     | OT_INT      | None            | 1 * 1-line      |\n"
"| level           |                 |                 | major iteration |\n"
"|                 |                 |                 | log             |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Major step      | OT_DOUBLE         | None            | 2               |\n"
"| limit           |                 |                 |                 |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Minor           | OT_DOUBLE         | None            | 1.0e-6 * for    |\n"
"| feasibility     |                 |                 | satisfying the  |\n"
"| tolerance       |                 |                 | QP bounds       |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Minor           | OT_INT      | None            | 500 * or 3m if  |\n"
"| iterations      |                 |                 | that is more    |\n"
"| limit           |                 |                 |                 |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Minor print     | OT_INT      | None            | 1 * 1-line      |\n"
"| level           |                 |                 | minor iteration |\n"
"|                 |                 |                 | log             |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| New basis file  | OT_INT      | None            | 0 * output      |\n"
"|                 |                 |                 | basis map       |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| New superbasics | OT_INT      | None            | 99 * controls   |\n"
"| limit           |                 |                 | early           |\n"
"|                 |                 |                 | termination of  |\n"
"|                 |                 |                 | QPs             |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Old basis file  | OT_INT      | None            | 0 * input basis |\n"
"|                 |                 |                 | map             |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Partial price   | OT_INT      | None            | 1 * 10 for      |\n"
"|                 |                 |                 | large LPs       |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Penalty         | OT_DOUBLE         | None            | 0.0 * initial   |\n"
"| parameter       |                 |                 | penalty         |\n"
"|                 |                 |                 | parameter       |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Pivot tolerance | OT_DOUBLE         | None            | 3.7e-11 * e^2/3 |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Print frequency | OT_INT      | None            | 100 * minor     |\n"
"|                 |                 |                 | iterations log  |\n"
"|                 |                 |                 | on Print file   |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Proximal point  | OT_INT      | None            | 1 * satisfies   |\n"
"| method          |                 |                 | linear          |\n"
"|                 |                 |                 | constraints     |\n"
"|                 |                 |                 | near x0         |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Punch file      | OT_INT      | None            | 0 * output      |\n"
"|                 |                 |                 | Insert data     |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| QPSolver        | OT_STRING       | None            | Cholesky *      |\n"
"|                 |                 |                 | default         |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Reduced Hessian | OT_INT      | None            | 2000 * or       |\n"
"| dimension       |                 |                 | Superbasics     |\n"
"|                 |                 |                 | limit if that   |\n"
"|                 |                 |                 | is less         |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Save frequency  | OT_INT      | None            | 100 * save      |\n"
"|                 |                 |                 | basis map       |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Scale option    | OT_INT      | None            | 1 * linear      |\n"
"|                 |                 |                 | constraints and |\n"
"|                 |                 |                 | variables       |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Scale tolerance | OT_DOUBLE         | None            | 0.900           |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Solution        | OT_STRING       | None            | Yes * on the    |\n"
"|                 |                 |                 | Print file      |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Solution file   | OT_INT      | None            | 0 * different   |\n"
"|                 |                 |                 | from printed    |\n"
"|                 |                 |                 | solution        |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Sticky          | OT_STRING       | None            | No * Yes makes  |\n"
"| parameters      |                 |                 | parameter       |\n"
"|                 |                 |                 | values persist  |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Summary         | OT_INT      | None            | 100 * minor     |\n"
"| frequency       |                 |                 | iterations log  |\n"
"|                 |                 |                 | on Summary file |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Superbasics     | OT_INT      | None            | n1 + 1 * n1 =   |\n"
"| limit           |                 |                 | number of       |\n"
"|                 |                 |                 | nonlinear       |\n"
"|                 |                 |                 | variables       |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| System          | OT_STRING       | None            | No * Yes prints |\n"
"| information     |                 |                 | more system     |\n"
"|                 |                 |                 | information     |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Timing level    | OT_INT      | None            | 3 * print cpu   |\n"
"|                 |                 |                 | times           |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Unbounded       | OT_DOUBLE         | None            | 1.000e+15       |\n"
"| objective       |                 |                 |                 |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Unbounded step  | OT_DOUBLE         | None            | 1.000e+18       |\n"
"| size            |                 |                 |                 |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Verify level    | OT_INT      | None            | 0 * cheap check |\n"
"|                 |                 |                 | on gradients    |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| Violation limit | OT_DOUBLE         | None            | 10.0 * unscaled |\n"
"|                 |                 |                 | constraint      |\n"
"|                 |                 |                 | violation limit |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| detect_linear   | OT_BOOL      | True            | Make an effort  |\n"
"|                 |                 |                 | to treat linear |\n"
"|                 |                 |                 | constraints and |\n"
"|                 |                 |                 | linear          |\n"
"|                 |                 |                 | variables       |\n"
"|                 |                 |                 | specially.      |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| print file      | OT_STRING       | None            | n/a             |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| print_time      | OT_BOOL      | True            | print           |\n"
"|                 |                 |                 | information     |\n"
"|                 |                 |                 | about execution |\n"
"|                 |                 |                 | time            |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| specs file      | OT_STRING       | None            | n/a             |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| start           | OT_STRING       | Cold            |                 |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"| summary         | OT_BOOL      | True            | n/a             |\n"
"+-----------------+-----------------+-----------------+-----------------+\n"
"\n"
"\n"
">List of available monitors\n"
"\n"
"+-----------+\n"
"|    Id     |\n"
"+===========+\n"
"| eval_nlp  |\n"
"+-----------+\n"
"| setup_nlp |\n"
"+-----------+\n"
"\n"
"\n"
">List of available stats\n"
"\n"
"+----------------+\n"
"|       Id       |\n"
"+================+\n"
"| iter_count     |\n"
"+----------------+\n"
"| iterations     |\n"
"+----------------+\n"
"| n_callback_fun |\n"
"+----------------+\n"
"| n_eval_grad_f  |\n"
"+----------------+\n"
"| n_eval_jac_g   |\n"
"+----------------+\n"
"| return_status  |\n"
"+----------------+\n"
"| t_callback_fun |\n"
"+----------------+\n"
"| t_eval_grad_f  |\n"
"+----------------+\n"
"| t_eval_jac_g   |\n"
"+----------------+\n"
"| t_mainloop     |\n"
"+----------------+\n"
"\n"
"\n"
"\n"
"\n"
;
