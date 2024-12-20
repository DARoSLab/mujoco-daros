#include "eigenHelper.hpp"
Eigen::MatrixXf dmToEigenf(casadi::DM m)
{
    int rows = m.size1();
    int cols = m.size2();
    Eigen::MatrixXf out = Eigen::MatrixXf::Zero(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            out(i, j) = m(i, j).scalar();
        }
    }
    return out;

}

Eigen::MatrixXd dmToEigen(casadi::DM m)
{
    int rows = m.size1();
    int cols = m.size2();
    Eigen::MatrixXd out = Eigen::MatrixXd::Zero(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            out(i, j) = m(i, j).scalar();
        }
    }
    return out;

}

Eigen::VectorXf dmToEigenVectorf (casadi::DM m){
    int rows = m.size1();
    int cols = m.size2();
    Eigen::VectorXf out = Eigen::VectorXf::Zero(rows*cols);
    for (int i = 0; i < rows*cols; i++)
    {
        out(i) = m(i).scalar();
    }
    return out;
}

Eigen::VectorXd dmToEigenVector (casadi::DM m){
    int rows = m.size1();
    int cols = m.size2();
    Eigen::VectorXd out = Eigen::VectorXd::Zero(rows*cols);
    for (int i = 0; i < rows*cols; i++)
    {
        out(i) = m(i).scalar();
    }
    return out;
}

casadi::DM EigenTodm(Eigen::MatrixXd matrix){
    size_t rows = matrix.rows();
    size_t cols = matrix.cols();

    casadi::DM casadi_matrix = casadi::DM::zeros(rows,cols);

    std::memcpy(casadi_matrix.ptr(), matrix.data(), sizeof(double)*rows*cols);

    return casadi_matrix;
}

Eigen::MatrixXd arrayToEigen(double m[], int row , int col){

    Eigen::MatrixXd e(row,col);
    for (int i = 0; i < col; i++)
    {
        for (int j = 0; j < row; j++)
        {
            e(j,i)=m[i*row+j];
        }

    }
    return e;
}

casadi::DM EigenVectorTodm(Eigen::VectorXd vec)
{
    int rows = static_cast<int>(vec.rows());
    // convert rows to intger type


    casadi::DM casadi_vec = casadi::DM::zeros(rows, 1);
    for (int i = 0; i < rows; i++)
    {
        casadi_vec(i, 0) = vec(i);
    }
    return casadi_vec;
}

casadi::DM EigenVectorfTodm(Eigen::VectorXf vec)
{
    int rows = static_cast<int>(vec.rows());
    // convert rows to intger type


    casadi::DM casadi_vec = casadi::DM::zeros(rows, 1);
    for (int i = 0; i < rows; i++)
    {
        casadi_vec(i, 0) = vec(i);
    }
    return casadi_vec;
}

casadi::DM EigenMatrixTodm(Eigen::MatrixXd matrix)
{
    int rows = static_cast<int>(matrix.rows());
    int cols = static_cast<int>(matrix.cols());
    casadi::DM casadi_matrix = casadi::DM::zeros(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            casadi_matrix(i, j) = matrix(i, j);
        }
    }
    return casadi_matrix;
}

casadi::DM EigenMatrixfTodm(Eigen::MatrixXf matrix)
{
    int rows = static_cast<int>(matrix.rows());
    int cols = static_cast<int>(matrix.cols());
    casadi::DM casadi_matrix = casadi::DM::zeros(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            casadi_matrix(i, j) = matrix(i, j);
        }
    }
    return casadi_matrix;
}

Eigen::Matrix3d yprToRotmat(double roll, double pitch, double yaw){
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return R;
}


Eigen::Matrix3d skew(Eigen::Vector3d v)
{
    Eigen::Matrix3d out;
    out <<  0, -v(2),   v(1),
         v(2),     0,  -v(0),
        -v(1),  v(0),      0;
    return out;
}

Eigen::MatrixXd get_N()
{
    Eigen::MatrixXd out(9, 3);
    out << 0, 0, 0,
           0, 0, 1,
           0,-1, 0,
           0, 0,-1,
           0, 0, 0,
           1, 0, 0,
           0, 1, 0,
          -1, 0, 0,
           0, 0, 0;
    return out;
}

Eigen::MatrixXd get_D(Eigen::Vector3d v){
    Eigen::MatrixXd out(9, 3);
    double d = v(0); double e = v(1); double f = v(2);
    out << 0, 0, 0,
           e,-d, 0,
           f, 0,-d,
          -e, d,-0,
           0, 0, 0,
           0, f,-e,
          -f, 0, d,
           0,-f, e,
           0, 0, 0;
    return out;
}

Eigen::MatrixXd get_F(Eigen::Vector3d v){
    Eigen::MatrixXd out(3, 9);
    double a = v(0); double b = v(1); double c = v(2);
    out << a, b, c, 0, 0, 0, 0, 0, 0,
           0, 0, 0, a, b, c, 0, 0, 0,
           0, 0, 0, 0, 0, 0, a, b, c;
    return out;
}

Eigen::VectorXd vectorize(Eigen::MatrixXd m){
    int rows = static_cast<int>(m.rows());
    int cols = static_cast<int>(m.cols());
    Eigen::VectorXd out(rows*cols);
    for (int i = 0; i < cols; i++)
    {
        for (int j = 0; j < rows; j++)
        {
            out(j+ i* rows)= m(j,i);
        }
    }
    return out;
}

casadi::DM cross2d(casadi::DM x, casadi::DM y){
    return x(0)*y(1) - x(1)*y(0);

}

Eigen::Matrix3d euler2Rot(const double roll,
                          const double pitch,
                          const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q.matrix();
}

Coeffs eta_co_R(Eigen::Matrix3d Rop, Eigen::Vector3d wop, double dt){
    Eigen::MatrixXd N = get_N();
    Eigen::MatrixXd invN = N.completeOrthogonalDecomposition().pseudoInverse().eval();

    Eigen::MatrixXd C_eta = Eigen::KroneckerProduct<Eigen::Matrix3d, Eigen::Matrix3d>(Eigen::Matrix3d::Identity(), Rop* skew(wop)).eval()*N
               + Eigen::KroneckerProduct<Eigen::Matrix3d, Eigen::Matrix3d>(Eigen::Matrix3d::Identity(), Rop).eval()*get_D(wop);
    Eigen::MatrixXd C_w   = Eigen::KroneckerProduct<Eigen::Matrix3d, Eigen::Matrix3d>(Eigen::Matrix3d::Identity(), Rop).eval() * N;

    Eigen::VectorXd C_c   = vectorize(Rop*skew(wop)) 
               - Eigen::KroneckerProduct<Eigen::Matrix3d, Eigen::Matrix3d>(Eigen::Matrix3d::Identity(), Rop).eval()*N*wop;

    Eigen::Matrix3d CE_eta = Eigen::Matrix3d::Identity() 
              + invN * dt * Eigen::KroneckerProduct<Eigen::Matrix3d, Eigen::Matrix3d>(Eigen::Matrix3d::Identity(), Rop.transpose()).eval() * C_eta;
    
    Eigen::Matrix3d CE_w = invN * dt * Eigen::KroneckerProduct<Eigen::Matrix3d, Eigen::Matrix3d>(Eigen::Matrix3d::Identity(), Rop.transpose()).eval() * C_w;
    Eigen::Vector3d CE_c = invN * dt * Eigen::KroneckerProduct<Eigen::Matrix3d, Eigen::Matrix3d>(Eigen::Matrix3d::Identity(), Rop.transpose()).eval() * C_c;
    
    Coeffs out;
    out.CE_eta << CE_eta; out.CE_w << CE_w; out.CE_c << CE_c;
    return out;
}
Woeffs eta_co_w(Eigen::Vector3d Xop, Eigen::Matrix3d Rop, Eigen::Vector3d wop, Eigen::VectorXd fop, double dt, Eigen::MatrixXd pf, Eigen::Matrix3d J, Eigen::Matrix3d Jinv)
{
    Eigen::MatrixXd N = get_N();

    Eigen::Vector3d r1 = pf.col(0) - Xop;
    Eigen::Vector3d r2 = pf.col(1) - Xop;
    Eigen::Vector3d r3 = pf.col(2) - Xop;
    Eigen::Vector3d r4 = pf.col(3) - Xop;
    // Eigen::Vector3d dc1 = dC.col(0);
    // Eigen::Vector3d dc2 = dC.col(1);
    // Eigen::Vector3d dc3 = dC.col(2);
    // Eigen::Vector3d dc4 = dC.col(3);

    Eigen::MatrixXd tempR(3, 12);
    tempR <<  skew(r1), skew(r2), skew(r3), skew(r4);
    Eigen::Vector3d Mop = tempR * fop;
    Eigen::Matrix3d temp_J_w = skew(J*wop) - skew(wop)*J;

    Eigen::MatrixXd tempEyes(3, 12); tempEyes<< Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity();
    Eigen::Vector3d sum_fop = tempEyes * fop;

    Eigen::MatrixXd Cx = Rop.transpose() * skew(sum_fop);
    Eigen::MatrixXd Ceta = get_F(Rop.transpose()*Mop) * N - temp_J_w * skew(wop);
    Eigen::MatrixXd Cw = temp_J_w;

    Eigen::MatrixXd Cu = Rop.transpose() * tempR;   // missing the part I added in matlab
    Eigen::Vector3d Cc = -skew(wop)*J*wop + Rop.transpose()* Mop - temp_J_w * wop - Cx * Xop; // + Rop.transpose()*
    Eigen::Matrix3d Cw_x = dt * Jinv * Cx;
    Eigen::Matrix3d Cw_eta = dt * Jinv * Ceta;
    Eigen::Matrix3d Cw_w = dt * Jinv * Cw + Eigen::Matrix3d::Identity();
    Eigen::MatrixXd Cw_u = dt * Jinv * Cu;
    Eigen::Vector3d Cw_c = dt * Jinv * Cc;

    Woeffs out; out.Cw_u = Eigen::MatrixXd::Zero(3,12);
    out.Cw_c << Cw_c; 
    out.Cw_eta << Cw_eta; 
    out.Cw_u << Cw_u; 
    out.Cw_w << Cw_w; 
    out.Cw_x << Cw_x;

    return out;
}

// casadi::Function conv_3by3(){
//     casadi::MX in1 = casadi::MX::sym("x", 9);
//     casadi::MX out1 = casadi::MX::zeros(3, 3);
//     out1()
// }

 casadi::Function expm_casadi(double dt){
    casadi::MX in1 = casadi::MX::sym("x", 3, 3);

    casadi::MX exp_omega = casadi::MX::eye(3);
    casadi::MX power_term = casadi::MX::eye(3);
    double factorial_term = 1.0;
    int num_terms = 3;
    for (int k = 1; k <= num_terms; ++k) {
        // Update the power term and factorial
        power_term = mtimes(power_term, dt * in1);
        factorial_term *= k;

        // Add the current term to the exponential matrix
        exp_omega = exp_omega + power_term / factorial_term;
    }

    casadi::Function constraint("expm_casadi", {in1}, {exp_omega});
    return constraint;
 }

 casadi::Function rpy2rot_casadi(){
    casadi::MX in1 = casadi::MX::sym("x", 3);
    casadi::MX roll = in1(0);
    casadi::MX pitch = in1(1);
    casadi::MX yaw = in1(2);

    casadi::MX R = casadi::MX::zeros(3, 3);
    R(0, 0) = cos(roll) * cos(pitch);
    R(0, 1) = cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw);
    R(0, 2) = cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw);
    R(1, 0) = sin(roll) * cos(pitch);
    R(1, 1) = sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw);
    R(1, 2) = sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw);
    R(2, 0) = -sin(pitch);
    R(2, 1) = cos(pitch) * sin(yaw);
    R(2, 2) = cos(pitch) * cos(yaw);

    casadi::Function constraint("rpy2rot", {in1}, {R});
    return constraint;
 }

 casadi::Function local_angular_to_euler_rate_casadi(){
    casadi::MX in1 = casadi::MX::sym("x", 3);
    casadi::MX x = in1(0);
    casadi::MX y = in1(1);
    casadi::MX z = in1(2);
    casadi::MX sy = sin(y);
    casadi::MX cy = cos(y);
    casadi::MX sz = sin(z);
    casadi::MX cz = cos(z);
    casadi::MX cz_cy = cz / cy;
    casadi::MX sz_cy = sz / cy;
    casadi::MX M = casadi::MX::zeros(3, 3);
    M(0, 0) = cz_cy;        M(0, 1) = -sz_cy;       M(0, 2) = 0;
    M(1, 0) = sz;           M(1, 1) = cz;           M(1, 2) = 0;
    M(2, 0) = -sy * cz_cy;  M(2, 1) = sy * sz_cy;   M(2, 2) = 1;
    casadi::Function constraint("local_angular_to_euler_rate", {in1}, {M});
    return constraint;
 }

  casadi::Function world_angular_to_euler_rate_casadi(){
    casadi::MX in1 = casadi::MX::sym("x", 3);
    casadi::MX x = in1(0);
    casadi::MX y = in1(1);
    casadi::MX z = in1(2);
    casadi::MX sy = sin(y);
    casadi::MX cy = cos(y);
    casadi::MX sz = sin(z);
    casadi::MX cz = cos(z);
    casadi::MX cz_cy = cz / cy;
    casadi::MX sz_cy = sz / cy;
    casadi::MX M = casadi::MX::zeros(3, 3);
    // M(0, 0) = cz_cy;        M(0, 1) = -sz_cy;       M(0, 2) = 0;
    // M(1, 0) = sz;           M(1, 1) = cz;           M(1, 2) = 0;
    // M(2, 0) = -sy * cz_cy;  M(2, 1) = sy * sz_cy;   M(2, 2) = 1;
    M(0, 0) = cz;           M(0, 1) = sz;          M(0, 2) = 0;
    M(1, 0) = -sz;          M(1, 1) = cz;          M(1, 2) = 0;
    M(2, 0) = 0;            M(2, 1) = 0;           M(2, 2) = 1;
    casadi::Function constraint("local_angular_to_euler_rate", {in1}, {M});
    return constraint;
 }