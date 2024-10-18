#include "WBDC.hpp"
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

template <typename T>
WBDC<T>::WBDC(size_t num_qdot, const std::vector<ContactSpec<T>*>* contact_list,
        const std::vector<Task<T>*>* task_list)
    : WBC<T>(num_qdot), _dim_floating(6) {
        _contact_list = contact_list;
        _task_list = task_list;

        _eye = DMat<T>::Identity(this->num_qdot_, this->num_qdot_);
        _eye_floating = DMat<T>::Identity(_dim_floating, _dim_floating);
    }

template <typename T>
void WBDC<T>::UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
        const DVec<T>& cori, const DVec<T>& grav,
        void* extra_setting) {
    this->A_ = A;
    this->Ainv_ = Ainv;
    this->cori_ = cori;
    this->grav_ = grav;
    this->b_updatesetting_ = true;

    (void)extra_setting;
}

template <typename T>
bool WBDC<T>::_CheckNullSpace(const DMat<T> & Npre){
    DMat<T> M_check = this->Sv_ * this->A_ * Npre;
    Eigen::JacobiSVD<DMat<T> > svd(M_check, Eigen::ComputeThinU | Eigen::ComputeThinV);
    pretty_print(svd.singularValues(), std::cout, "svd singular value");

    for(int i(0); i<svd.singularValues().rows(); ++i){
        if(svd.singularValues()[i] > 0.00001) { 
            printf("non singular!!\n"); 
            pretty_print(svd.singularValues(), std::cout, "svd singular value");
            return false;
        }else{
            printf("small enough singular value: %f\n", svd.singularValues()[i]);
        }
    }
    return true;
}

template <typename T>
void WBDC<T>::MakeTorque(DVec<T>& cmd, void* extra_input) {
    _PrintDebug(1);    
    if(!this->b_updatesetting_) { printf("[Wanning] WBDC setting is not done\n"); }
    if(extra_input) _data = static_cast<WBIC_ExtraData<T>* >(extra_input);

    _PrintDebug(2);    
    // Contact Setting 
    _ContactBuilding();

    DMat<T> Jt, JtPre, JtPreBar, JcBar;
    DVec<T> JtDotQdot, xddot, qddot_pre;
    DMat<T> Npre;

    if (_dim_rf > 0) {
        this->_WeightedInverse(_Jc, this->Ainv_, JcBar);
        qddot_pre = JcBar * ( - _JcDotQdot );
        Npre = _eye - JcBar* _Jc;

    } else {
        qddot_pre = DVec<T>::Zero(this->num_qdot_);
        Npre = _eye;
    }

    _PrintDebug(4);    
    // First Task Check
    Task<T>* task = (*_task_list)[0];

    _PrintDebug(5);    
    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    task->getCommand(xddot);
    dim_first_task_ = task->getDim();

    //pretty_print(Jt, std::cout, "Jt");
    //pretty_print(Npre, std::cout, "Npre");
    JtPre = Jt * Npre;
    this->_WeightedInverse(JtPre, this->Ainv_, JtPreBar);
    Npre = Npre * (_eye - JtPreBar * JtPre);

    _PrintDebug(6);    
    //_CheckNullSpace(Npre);
    // Optimization
    _PrintDebug(7);    
    // Set inequality constraints
    _OptimizationPreparation();
    _PrintDebug(7.2);    
    _SetInEqualityConstraint();
    _PrintDebug(7.5);    
    // Set equality constraints
    DMat<T> dyn_CE(_dim_eq_cstr, _dim_opt);
    DVec<T> dyn_ce0(_dim_eq_cstr);
    dyn_CE.block(0,0, _dim_eq_cstr, dim_first_task_) = this->Sv_ * this->A_ * JtPreBar;
    dyn_CE.block(0, dim_first_task_, _dim_eq_cstr, _dim_rf) = -this->Sv_ * _Jc.transpose();
    dyn_ce0 = this->Sv_ * (this->A_ * (JtPreBar * (xddot - JtDotQdot - Jt * qddot_pre) + qddot_pre)
            + this->cori_ + this->grav_);
    for(size_t i(0); i< _dim_eq_cstr; ++i){
        for(size_t j(0); j<_dim_opt; ++j){
            CE[j][i] = dyn_CE(i,j);
        }
        ce0[i] = dyn_ce0[i];
    }
    // Optimization
    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
    DVec<T> delta(dim_first_task_);
    for(size_t i(0); i<dim_first_task_; ++i) { delta[i] = z[i]; }
    qddot_pre = qddot_pre + JtPreBar * (xddot + delta - JtDotQdot - Jt * qddot_pre);
    //pretty_print(xddot, std::cout, "xddot");
    //pretty_print(delta, std::cout, "delta");

    // First Qddot is found
    // Stack The last Task
    for(size_t i(1); i<(*_task_list).size(); ++i){
        task = (*_task_list)[i];

        //if(!task->IsTaskSet()){ printf("%d th task is not set!\n", i); exit(0); }
        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        task->getCommand(xddot);

        JtPre = Jt * Npre;
        this->_WeightedInverse(JtPre, this->Ainv_, JtPreBar);

        qddot_pre = qddot_pre + JtPreBar * (xddot - JtDotQdot - Jt * qddot_pre);

        Npre = Npre * (DMat<T>::Identity(this->num_qdot_, this->num_qdot_) 
                - JtPreBar * JtPre);
    }

    _GetSolution(qddot_pre, cmd);

    _data->_opt_result = DVec<T>(_dim_opt);
    for(size_t i(0); i<_dim_opt; ++i){
        _data->_opt_result[i] = z[i];
    }
    //std::cout << "f: " << f << std::endl;
    //std::cout << "x: " << z << std::endl;
    //std::cout << "cmd: "<<cmd<<std::endl;
    //dynacore::pretty_print(this->Sv_, std::cout, "Sf");
    //dynacore::pretty_print(qddot_pre, std::cout, "qddot_pre");
    //dynacore::pretty_print(Nci_, std::cout, "Nci");
    //DVec<T> eq_check = dyn_CE * _data->opt_result_;
    //dynacore::pretty_print(dyn_ce0, std::cout, "dyn ce0");
    //dynacore::pretty_print(eq_check, std::cout, "eq_check");

    //dynacore::pretty_print(Jt, std::cout, "Jt");
    //dynacore::pretty_print(JtDotQdot, std::cout, "Jtdotqdot");
    //dynacore::pretty_print(xddot, std::cout, "xddot");


    //printf("G:\n");
    //std::cout<<G<<std::endl;
    //printf("g0:\n");
    //std::cout<<g0<<std::endl;

    //printf("CE:\n");
    //std::cout<<CE<<std::endl;
    //printf("ce0:\n");
    //std::cout<<ce0<<std::endl;

    //printf("CI:\n");
    //std::cout<<CI<<std::endl;
    //printf("ci0:\n");
    //std::cout<<ci0<<std::endl;


    // if(f > 1.e5){
    //   std::cout << "f: " << f << std::endl;
    //   std::cout << "x: " << z << std::endl;
    //   std::cout << "cmd: "<<cmd<<std::endl;

       //printf("G:\n");
       //std::cout<<G<<std::endl;
    //   printf("g0:\n");
    //   std::cout<<g0<<std::endl;

    //   printf("CE:\n");
    //   std::cout<<CE<<std::endl;
    //   printf("ce0:\n");
    //   std::cout<<ce0<<std::endl;

    //   printf("CI:\n");
    //   std::cout<<CI<<std::endl;
    //   printf("ci0:\n");
    //   std::cout<<ci0<<std::endl;
    // }

}

template <typename T>
void WBDC<T>::_SetInEqualityConstraint(){
    DMat<T> dyn_CI(_dim_ieq_cstr, _dim_opt); dyn_CI.setZero();
    DVec<T> dyn_ci0(_dim_ieq_cstr);

    dyn_CI.block(0, dim_first_task_, _dim_Uf, _dim_rf) = _Uf;
    dyn_ci0 = _Uf_ieq_vec;

    for(size_t i(0); i< _dim_ieq_cstr; ++i){
        for(size_t j(0); j<_dim_opt; ++j){
            CI[j][i] = dyn_CI(i,j);
        }
        ci0[i] = -dyn_ci0[i];
    }
    // dynacore::pretty_print(dyn_CI, std::cout, "WBDC: CI");
    // dynacore::pretty_print(dyn_ci0, std::cout, "WBDC: ci0");
}

template <typename T>
void WBDC<T>::_ContactBuilding(){
    // Dimension
    _dim_rf = 0;
    _dim_Uf = 0;  // Dimension of inequality constraint
    for (size_t i(0); i < (*_contact_list).size(); ++i) {
        _dim_rf += (*_contact_list)[i]->getDim();
        _dim_Uf += (*_contact_list)[i]->getDimRFConstraint();
    }
    //printf("dim rf, uf: %lu, %lu\n", _dim_rf, _dim_Uf);
    _Jc = DMat<T>(_dim_rf, this->num_qdot_);
    _JcDotQdot = DVec<T>(_dim_rf);
    _Uf = DMat<T>(_dim_Uf, _dim_rf);
    _Uf.setZero();
    _Uf_ieq_vec = DVec<T>(_dim_Uf);


    DMat<T> Uf;
    DVec<T> Uf_ieq_vec;
    // Initial
    DMat<T> Jc;
    DVec<T> JcDotQdot;
    size_t dim_accumul_rf, dim_accumul_uf;
    (*_contact_list)[0]->getContactJacobian(Jc);
    (*_contact_list)[0]->getJcDotQdot(JcDotQdot);
    (*_contact_list)[0]->getRFConstraintMtx(Uf);
    (*_contact_list)[0]->getRFConstraintVec(Uf_ieq_vec);

    dim_accumul_rf = (*_contact_list)[0]->getDim();
    dim_accumul_uf = (*_contact_list)[0]->getDimRFConstraint();

    _Jc.block(0, 0, dim_accumul_rf, this->num_qdot_) = Jc;
    _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
    _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
    _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;

    size_t dim_new_rf, dim_new_uf;

    for (size_t i(1); i < (*_contact_list).size(); ++i) {
        (*_contact_list)[i]->getContactJacobian(Jc);
        (*_contact_list)[i]->getJcDotQdot(JcDotQdot);

        dim_new_rf = (*_contact_list)[i]->getDim();
        dim_new_uf = (*_contact_list)[i]->getDimRFConstraint();

        // Jc append
        _Jc.block(dim_accumul_rf, 0, dim_new_rf, this->num_qdot_) = Jc;

        // JcDotQdot append
        _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

        // Uf
        (*_contact_list)[i]->getRFConstraintMtx(Uf);
        _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

        // Uf inequality vector
        (*_contact_list)[i]->getRFConstraintVec(Uf_ieq_vec);
        _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

        dim_accumul_rf += dim_new_rf;
        dim_accumul_uf += dim_new_uf;
    }
    // dynacore::pretty_print(_Jc, std::cout, "WBDC: Jc");
    // dynacore::pretty_print(_JcDotQdot, std::cout, "WBDC: JcDot Qdot");
    // dynacore::pretty_print(Uf_, std::cout, "WBDC: Uf");
}

template <typename T>
void WBDC<T>::_GetSolution(const DVec<T> & qddot, DVec<T> & cmd){
    DVec<T> Fr(_dim_rf);
    for(size_t i(0); i<_dim_rf; ++i) Fr[i] = z[i + dim_first_task_];
    DVec<T> tot_tau = this->A_ * qddot + this->cori_ + this->grav_ - (_Jc).transpose() * Fr;

    cmd = tot_tau.tail(this->num_act_joint_);
    //DMat<T> UNci = this->Sa_ * Nci_;
    //DMat<T> UNciBar;
    // only work in 3D case
    //DMat<T> UNci_trc = UNci.block(0,6, this->num_act_joint_, this->num_qdot_-6);
    //DVec<T> tot_tau_trc = tot_tau.tail(num_qdot_-6);
    //DMat<T> UNciBar_trc;
    //DMat<T> eye(num_qdot_-6, num_qdot_-6);
    //eye.setIdentity();
    //this->_WeightedInverse(UNci_trc, eye, UNciBar_trc);
    //DMat<T> Ainv_trc = Ainv_.block(6,6, num_qdot_-6, num_qdot_-6);
    //_WeightedInverse(UNci_trc, Ainv_trc, UNciBar_trc, 0.0001);
    //cmd = UNciBar_trc.transpose() * tot_tau_trc;
    //cmd = (UNci_trc.inverse()).transpose()* tot_tau_trc;
    //dynacore::pretty_print(UNci, std::cout, "UNci");
    //dynacore::pretty_print(UNci_trc, std::cout, "UNci+trc");
    //dynacore::pretty_print(UNciBar_trc, std::cout, "UNciBar_trc");
    //dynacore::pretty_print(tot_tau_trc, std::cout, "tot tau trc");
    //_WeightedInverse(UNci, Ainv_, UNciBar);
    //cmd = UNciBar.transpose() * tot_tau;
    // dynacore::pretty_print(result, std::cout, "opt result");
    //dynacore::pretty_print(tot_tau, std::cout, "tot tau result");
    //dynacore::pretty_print(cmd, std::cout, "final command");
}

template <typename T>
void WBDC<T>::_OptimizationPreparation(){

    _dim_opt = _dim_rf + dim_first_task_; 
    _dim_eq_cstr = 6;
    _dim_ieq_cstr = _dim_Uf; 

    G.resize(_dim_opt, _dim_opt); 
    g0.resize(_dim_opt);
    CE.resize(_dim_opt, _dim_eq_cstr);
    ce0.resize(_dim_eq_cstr);
    CI.resize(_dim_opt, _dim_ieq_cstr);
    ci0.resize(_dim_ieq_cstr);

    for(size_t i(0); i<_dim_opt; ++i){
        for(size_t j(0); j<_dim_opt; ++j){
            G[i][j] = 0.;
        }
        g0[i] = 0.;
    }
    // Set Cost
    size_t idx_offset(0);
    // TODO
    for (size_t i(0); i < dim_first_task_; ++i) {
        G[i + idx_offset][i + idx_offset] = 10000.;// _data->_W_floating[i];
    }
    idx_offset += _dim_floating;
    pretty_print(_data->_W_rf, std::cout, "W rf");
    printf("%zu\n", _dim_rf);
    for (size_t i(0); i < _dim_rf; ++i) {
        G[i + idx_offset][i + idx_offset] = _data->_W_rf[i];
    }
    //for (int i(0); i < _dim_opt; ++i){
    //G[i][i] = _data->cost_weight[i];
    //}
}

template class WBDC<double>;
template class WBDC<float>;
