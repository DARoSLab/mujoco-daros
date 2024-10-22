#include "optimizeStand.hpp"
#include <iostream>  
#include <typeinfo> 
#include "Utilities/orientation_tools.h"
OptimizeStand::OptimizeStand(){
    p_opts["expand"] = true;
    solverName = "ipopt";
    s_opts["max_iter"] = 200;
    s_opts["linear_solver"] = "ma27";
    // s_opts["constr_viol_tol"] = 0.001;
    // s_opts["acceptable_tol"] = 0.00001;
}

void OptimizeStand::init(const FloatingBaseModel<float> * model, TelloParameters* p){

    if (p->CAS_opt_details > 0) {
        s_opts["print_level"] = 5;
    } else {
        s_opts["print_level"] = 0;
        s_opts["sb"] = "yes";
        p_opts["print_time"] = 0;
    }
    pred_hor = p->CAS_pred_hor;
    DMat<float> mass_matrix = model->getMassMatrix();
    m    = static_cast<double>(mass_matrix(3,3));
    gravity  = EigenVectorTodm(p->gravity);

    opti = Opti();    

    X    = opti.variable(18,pred_hor+1); // state trajectory
    U    = opti.variable(12, pred_hor); // control trajectory
    X0   = opti.parameter(18,1); // initial state
    Loc  = opti.parameter(12,1); // location of the contact points
    Contact = opti.parameter(4, pred_hor); // contact sequence
    Ref  = opti.parameter(18,1); // reference state

    double dt   = p->CAS_dt;
    double mu   = p->mu;
    gravity  = EigenVectorTodm(p->gravity);
    DM mass_m   = EigenMatrixfTodm(mass_matrix.block(0,0, 3,3));
    auto x_des = Ref(Slice(0,3), all);
    auto v_des = Ref(Slice(3,6), all);
    auto w_des = Ref(Slice(6,9), all);
    auto R_des = Ref(Slice(9,18),all);

    DM Q_X  =  DM::diag(EigenVectorTodm(p->CAS_Q_X));
    DM Q_Xd =  DM::diag(EigenVectorTodm(p->CAS_Q_Xd));
    DM Q_R  =  DM::diag(EigenVectorTodm(p->CAS_Q_R));
    DM Q_W  =  DM::diag(EigenVectorTodm(p->CAS_Q_W));
    DM Q_U  =  DM::eye(12) * p->CAS_Q_U;

    int x_idx[] = {0,3,6,9};
    int y_idx[] = {1,4,7,10};
    int z_idx[] = {2,5,8,11};

    auto x    = X(Slice(0,3), all); 
    auto xd   = X(Slice(3,6), all);
    auto xw   = X(Slice(6,9), all);
    auto xR   = X(Slice(9,18),all);
    
    auto F_RHeel    = U(Slice(0,3), all); //  force at the right toe
    auto F_RToe     = U(Slice(3,6), all); //  force at the right heel
    auto F_LHeel    = U(Slice(6,9), all); //  force at the left toe
    auto F_LToe     = U(Slice(9,12),all); //  force at the left heel
    
    auto x0     = X0(Slice(0,3), all);
    auto xd0    = X0(Slice(3,6), all);
    auto w0     = X0(Slice(6,9), all);
    auto R0     = X0(Slice(9,18),all);
    
    auto rightHeelLoc   = Loc(Slice(0,3), all);
    auto rightToeLoc    = Loc(Slice(3,6), all);
    auto leftHeelLoc    = Loc(Slice(6,9), all);
    auto leftToeLoc     = Loc(Slice(9,12),all);

    casadi::Function expm_func =  expm_casadi(dt);

    cost = 0;
    // // starting from the second step, since the first step is given.
    for (int k = 1; k < pred_hor+1; k++){ 
        auto xk     = x (all, k);
        auto xdk    = xd(all, k);
        auto xwk    = xw(all, k);
        auto xRk    = xR(all, k);

        // 1: Ex
        auto E_x  = xk  - x_des;
        // 2: Exd
        auto E_xd = xdk - v_des;
        // 3: ER
        auto xRk_mat    = reshape(xRk, 3, 3);  // need to remember the reshape is column major
        auto R_des_mat  = reshape(R_des, 3, 3);
        auto eRot       = mtimes(R_des_mat.T(), xRk_mat);
        auto R_skew     = 0.5 * (eRot - eRot.T());
        auto E_R        = MX::vertcat({R_skew(2,1), R_skew(0,2), R_skew(1,0)});
        // 4: EW
        auto E_w = xwk;

        // 5: U
        auto fk = U(all, k-1);
        
        cost += mtimes(mtimes(    E_x.T(),Q_X),  E_x)
                +mtimes(mtimes(   E_xd.T(),Q_Xd), E_xd)
                +mtimes(mtimes(    E_R.T(),Q_R),  E_R)
                +mtimes(mtimes(    E_w.T(),Q_W),  E_w)
                +mtimes(mtimes(    fk.T(),Q_U),  fk);
    }
    opti.minimize(cost);

    // //initial constraint
    opti.subject_to(      x(all,0) == x0 );
    opti.subject_to(     xd(all,0) == xd0);
    opti.subject_to(     xw(all,0) == w0 );
    opti.subject_to(     xR(all,0) == R0 );

    for (int k = 0; k < pred_hor; k++){ 
        auto xk         = x (all, k);
        auto xdk        = xd(all, k);
        auto xwk        = xw(all, k);
        auto xRk        = xR(all, k);
        auto F_RToek    = F_RToe(all, k);
        auto F_RHeelk   = F_RHeel(all, k);
        auto F_LToek    = F_LToe(all, k);
        auto F_LHeelk   = F_LHeel(all, k);

        auto w_R_b = reshape(xRk, 3, 3);    // body in the world frame
        auto b_R_w = w_R_b.T();             // world in the body frame

        // Dyanmics
        auto xdd = 1/m * (F_RToek+F_RHeelk+F_LToek+F_LHeelk) + gravity;
        
        auto torque = cross(rightToeLoc - xk, F_RToek)  + cross(rightHeelLoc - xk, F_RHeelk)
                    + cross(leftToeLoc  - xk, F_LToek)  + cross(leftHeelLoc  - xk, F_LHeelk);
        auto I_omegadot = mtimes(b_R_w, torque) - cross(xwk, mtimes(mass_m, xwk));
        // 1: linear velocity
        opti.subject_to(xd(all, k+1) - xdk == xdd * dt); 
        // 2: linear position
        opti.subject_to( x(all, k+1) - xk  == xdk * dt);
        // 3: angular velocity, 
        opti.subject_to( mtimes(mass_m, xw(all, k+1) - xwk) == I_omegadot * dt);
        // 4: orientation
        auto omega_skew = skew(xwk);
        auto delta_R = expm_func(omega_skew)[0];
        auto xR_next = xR(all, k+1);
        // auto xR_next_mat = reshape(xR_next, 3, 3);
        auto temp_mat = mtimes(w_R_b, delta_R);
        opti.subject_to(reshape(temp_mat, 9, 1) == xR_next);

        // 5: friction constraint
        auto fk = U(all, k);
        for (int i = 0; i < 4; i++){
            opti.subject_to(-mu * fk(z_idx[i]) <= fk(x_idx[i]) <= mu * fk(z_idx[i]));
            opti.subject_to(-mu * fk(z_idx[i]) <= fk(y_idx[i]) <= mu * fk(z_idx[i]));
            opti.subject_to( fk(z_idx[i]) <= p->MAX_REACTION_FORCE);
        }   

        // 6: contact constraint
        auto contactk = Contact(all, k);
        for (int i = 0; i < 4; i++){
            opti.subject_to(  (1 - contactk(i)) * fk(z_idx[i]) == 0);
        }

        //7: minimum height
        // opti.subject_to(xk(2) >= 0.55);
        //8: maximum speed
        // opti.subject_to(xdk(0) <= 1);

    }
    opti.solver(solverName, p_opts, s_opts);
}


MPC_result OptimizeStand::optimize(const FloatingBaseModel<float> * model, TelloParameters* p, State &S, VectorXf ref_traj, MatrixXd contact_seq){
    
    MPC_result log;
    auto x    = X(Slice(0,3), all); 
    auto xd   = X(Slice(3,6), all);
    auto xw   = X(Slice(6,9), all);
    auto xR   = X(Slice(9,18),all);
    auto F_RHeel    = U(Slice(0,3), all); //  force at the right toe
    auto F_RToe     = U(Slice(3,6), all); //  force at the right heel
    auto F_LHeel    = U(Slice(6,9), all); //  force at the left toe
    auto F_LToe     = U(Slice(9,12),all); //  force at the left heel

    // initial guess
    opti.set_initial(x,    DM::repmat( S.x, 1, pred_hor+1));
    opti.set_initial(xd,   DM::repmat( S.xd, 1, pred_hor+1));
    opti.set_initial(xw,   DM::repmat( S.w, 1, pred_hor+1));
    opti.set_initial(xR,   DM::repmat( S.R, 1, pred_hor+1));
    DM init_state  = DM::vertcat({S.x, S.xd, S.w, S.R});
    DM init_loc    = S.contactLoc;
    DM ref         = EigenVectorfTodm(ref_traj);
    opti.set_value(X0, init_state);
    opti.set_value(Loc, init_loc);
    opti.set_value(Contact, EigenMatrixTodm(contact_seq));
    opti.set_value(Ref, ref);

    DM f_init = DM::zeros(3, pred_hor);
    for (int i = 0; i < pred_hor; i++){
        f_init(all, i) = -gravity * m / 4; 
    }

    opti.set_initial(F_RToe, f_init); opti.set_initial(F_RHeel, f_init);
    opti.set_initial(F_LToe, f_init); opti.set_initial(F_LToe, f_init);

    try {
        // actual solve
        auto sol = opti.solve();
        // double c = sol.value(cost).scalar();
    }
    catch (CasadiException  const&) {
        //show infeasibility
        std::cout << "infeasible" << std::endl;
        opti.debug().show_infeasibilities(0.000001);
        // std::cout<<opti.debug().g_describe(123)<<std::endl;
        // std::cout<<opti.debug().g_describe(18)<<std::endl;
    }
    log.x               = opti.debug().value(x);
    log.xd              = opti.debug().value(xd);
    log.xw              = opti.debug().value(xw);
    log.xR              = opti.debug().value(xR);
    log.U               = opti.debug().value(U);
    log.F_RToe          = opti.debug().value(F_RToe);
    log.F_RHeel         = opti.debug().value(F_RHeel);
    log.F_LToe          = opti.debug().value(F_LToe);
    log.F_LHeel         = opti.debug().value(F_LHeel);

    // std::cout << "x: \n" << log.x << std::endl;
    // std::cout << "xd: \n" << log.xd << std::endl;
    Eigen::MatrixXf R_all(3,pred_hor+1);
    for (int i = 0; i < pred_hor+1; i++){
        auto tempR = reshape(log.xR(all, i), 3, 3);
        Eigen::Matrix3f tempR_eig = dmToEigenf(tempR);
        // std::cout<< "tempR * tempR.T(): \n" << tempR_eig * tempR_eig.transpose() << std::endl;
        Vec3 <float> tempRPY = tempR_eig.eulerAngles(0,1,2);
        R_all.col(i) = tempRPY;
    }
    // std::cout<<"x: \n"<<log.x<<std::endl;
    // std::cout<<"xd: \n"<<log.xd<<std::endl;
    // std::cout << "RPY: \n" << R_all << std::endl;
    // std::cout<< "F_RHeel: \n" << log.F_RHeel << std::endl;
    // std::cout<< "F_RToe: \n" << log.F_RToe << std::endl;
    // std::cout<< "F_LHeel: \n" << log.F_LHeel << std::endl;
    // std::cout<< "F_LToe: \n" << log.F_LToe << std::endl;
    

    return log;

}