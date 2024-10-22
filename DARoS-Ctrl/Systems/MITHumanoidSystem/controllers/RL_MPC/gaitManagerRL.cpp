#include "gaitManagerRL.hpp"
#include <Utilities/Interpolation.h>

gaitManagerRL::gaitManagerRL(TelloParameters* parameters, int iteration_between_mpc, Eigen::VectorXd terrain):
terrain(terrain)
{
    p = parameters;
    _iteration_between_mpc = iteration_between_mpc;
    timeToPop = _iteration_between_mpc;
    for (int i = 0; i < 4; i++){
        _p0[i].setZero();
        _pf[i].setZero();
        _p[i].setZero();
        _v[i].setZero();
        _a[i].setZero();
    }
    contactTable = Eigen::MatrixXd::Ones(4, p->CAS_pred_hor);
    xlocationTable = Eigen::MatrixXd::Zero(4, p->CAS_pred_hor);
    zlocationTable = Eigen::MatrixXd::Zero(4, p->CAS_pred_hor);
    contactTableNext = Eigen::MatrixXd::Ones(4, p->CAS_pred_hor);
    xlocationTableNext = Eigen::MatrixXd::Zero(4, p->CAS_pred_hor);
    zlocationTableNext = Eigen::MatrixXd::Zero(4, p->CAS_pred_hor);
}

void gaitManagerRL::updates(){
    if (timeToPop == _iteration_between_mpc){
        // update mpctable 
        //concatenate contactTable and contactTableNext
        //easiest way to get the mpctable, we may need the contactTableNext or not depending on the time we are at. 
        contactTableCombined = Eigen::MatrixXd::Zero(4, contactTable.cols() + contactTableNext.cols());
        // std::cout<<"bug start here"<<std::endl;
        // std::cout<< "contactTable \n"<<contactTable<<std::endl;
        // std::cout<< "contactTableNext \n"<<contactTableNext<<std::endl;
        contactTableCombined << contactTable, contactTableNext;
        
        mpcTable = contactTableCombined.block(0, idxTable, 4, p->CAS_pred_hor);
        // std::cout<<"bug end here"<<std::endl;
        // std::cout<<"mpcTable: \n"<<mpcTable<<std::endl;
        xlocationTableCombined = Eigen::MatrixXd::Zero(4, xlocationTable.cols() + xlocationTableNext.cols());
        zlocationTableCombined = Eigen::MatrixXd::Zero(4, zlocationTable.cols() + zlocationTableNext.cols());
        
        xlocationTableCombined << xlocationTable, xlocationTableNext;
        // std::cout<<"xlocationTableCombined\n"<<xlocationTableCombined<<std::endl;
        // std::cout<<"zlocationTable\n"<<zlocationTable<<std::endl;
        // std::cout<<"zlocationTableNext\n"<<zlocationTableNext<<std::endl;
        zlocationTableCombined << zlocationTable, zlocationTableNext;
        // std::cout<<"zlocationTableCombined: \n"<<zlocationTableCombined<<std::endl;
        mpcLocTableX = xlocationTableCombined.block(0, idxTable, 4, p->CAS_pred_hor);
        // std::cout<<"mpcLocTableX: \n"<<mpcLocTableX<<std::endl;
        mpcLocTableZ = zlocationTableCombined.block(0, idxTable, 4, p->CAS_pred_hor);
        // std::cout<<"mpcLocTableZ: \n"<<mpcLocTableZ<<std::endl;
        // std::cout<<"xlocationTableCombined: \n"<<xlocationTableCombined<<std::endl;
        // std::cout<<"mpcLocTableX: \n"<<mpcLocTableX<<std::endl;
        // std::cout<<"mpcLocTableZ: \n"<<mpcLocTableZ<<std::endl;

    }
    for (int i = 0; i < 4; i++){
        if (mpcTable(i, 0) == 1){
            inAirTime[i] = 0;    // if it is in contact, reset the inAirTime
            swingPhase[i] = 0;   // if it is in contact, reset the swingPhase
            isSwing[i] = false;  // if it is in contact, set the isSwing to false
        } 
        else 
        {
            isSwing[i] = true;  // if it is in the air, set the isSwing to true
            
            // figure out how long the foot need to be in the air
            int count = 0;          // go all the way to the next contact
            for (int j = idxTable+1; j < contactTableCombined.cols(); j++){
                if (contactTableCombined(i, j) == 1)
                {
                    //record the contact location for the next contact
                    // std::cout<<"i: "<<i<<std::endl;
                    xlandingloc[i] = xlocationTableCombined(i, j);
                    prezlandingloc[i] = zlandingloc[i];
                    zlandingloc[i] = zlocationTableCombined(i, j);
                    // std::cout<<"xlandingloc[i]: "<<xlandingloc[i]<<std::endl;
                    // std::cout<<"zlandingloc[i]: "<<zlandingloc[i]<<std::endl;
                    break;
                }
                count += 1;
            }
            // total air_step is equal to how much 0 after the current 0 before the first 1 show up,
            // plus how long we are in the air also how long we will need to pop the current first 1
            int total_air_step = count  * _iteration_between_mpc + inAirTime[i] + timeToPop;
            swingTimes[i] = float(total_air_step) * p->controller_dt;
            // std::cout<<"total_air_step: "<<total_air_step<<std::endl;
            // std::cout<<"i: "<<i<<std::endl;
            swingPhase[i] = float(inAirTime[i]) / float(total_air_step);
            inAirTime[i] += 1;   // if it is in the air, add one to the inAirTime
        }
    }

}

int gaitManagerRL::getStates(){
    /* 
    there are three conditions that we need to consider, 
    1. we are at the end of the contactTable, then we will need to get new contactTable from the RL
    2. our current contactTable is not long enough to cover the CAS_pred_hor, then we will need to get new contactTable from the RL based on the predcition
    3. we don't need to get new contactTable from the RL, we can just use the current contactTable
    */ 
    if (idxTable >= contactTable.cols())
    {
        idxTable = 0;
        return 2;
    } 
    else if (idxTable + p->CAS_pred_hor >= contactTable.cols())
    {
        return 1;
    } 
    else 
    {
        return 0;
    }
}



void gaitManagerRL::setIterations(int iter){

    _iter = iter;

    timeToPop -= 1;
    if (timeToPop == 0){
        idxTable += 1;
        timeToPop = _iteration_between_mpc;
        
    }
}

ContactData gaitManagerRL::rl2dActToContactData(VectorXd act2d, float curr_contact_loc){
    ContactData cd;
    double desired_vel   = act2d(0); 
    // float  r_contact_loc = float(act2d(1));
    // double r_contact_dts = act2d(2);
    // double r_flight_dts  = act2d(3);
    // float  l_contact_loc = float(act2d(4));
    // double l_flight_dts  = act2d(5);
    // double l_contact_dts = act2d(6);

    float  r_contact_loc = float(act2d(1)) -0.02;
    float  l_contact_loc = float(act2d(2)) -0.02;
    int total_hor = 12;
    int r_contact_hor, r_flight_hor, l_flight_hor, l_contact_hor;
    if (act2d(3)<0){
        r_contact_hor = 3;
        r_flight_hor = 6;
        l_flight_hor = 3;
        l_contact_hor = 3;
    } else {
        // r_contact_hor = 2;
        // r_flight_hor = 8;
        // l_flight_hor = 4;
        // l_contact_hor = 4;
        r_contact_hor = 3;
        r_flight_hor = 6;
        l_flight_hor = 3;
        l_contact_hor = 6;
    }
    
    // int r_contact_hor = round(r_contact_dts / p->CAS_dt);
    // int r_flight_hor = round(r_flight_dts / p->CAS_dt);
    // int total_hor = r_contact_hor + r_flight_hor + r_contact_hor;  // +1 is for the contact after flight
    // int l_flight_hor = round(l_flight_dts / p->CAS_dt);
    // int l_contact_hor = round(l_contact_dts / p->CAS_dt);
    float curr_contact_loc_height = get_contact_height(curr_contact_loc);
    if (curr_contact_loc_height < 0){
        std::cout<<"curr_contact_loc smaller than 0"<<std::endl;
        // exit(0);
    }
    float rightContactLocWorld = r_contact_loc + curr_contact_loc;
    float rightContactLocWorldHeight = get_contact_height(rightContactLocWorld);
    if (rightContactLocWorldHeight < 0){
        std::cout<<"rightContactLocWorldHeight smaller than 0"<<std::endl;
        // exit(0);
    }
    // prevent l_contact_hor + l_flight_hor > total_hor
    l_contact_hor = min(l_contact_hor, total_hor - l_flight_hor);
    float leftContactLocWorld = l_contact_loc + curr_contact_loc;
    float leftContactLocWorldHeight = get_contact_height(leftContactLocWorld);
    if (leftContactLocWorldHeight < 0){
        std::cout<<"leftContactLocWorldHeight smaller than 0"<<std::endl;
        leftContactLocWorldHeight = 0.16;
        // exit(0);
    }
    cd.contactTable = Eigen::MatrixXd::Zero(4, total_hor);
    cd.xlocationTable = Eigen::MatrixXd::Zero(4, total_hor);
    cd.zlocationTable = Eigen::MatrixXd::Zero(4, total_hor);
    for (int i = 0; i< r_contact_hor; i++){
        cd.contactTable(0, i) = cd.contactTable(1, i) = 1;
        cd.xlocationTable(0, i) = cd.xlocationTable(1, i) = curr_contact_loc;
        cd.zlocationTable(0, i) = cd.zlocationTable(1, i) = curr_contact_loc_height;

    }
    for (int i = r_contact_hor + r_flight_hor; i < total_hor; i++){
        cd.contactTable(0, i) = cd.contactTable(1, i) = 1;
        cd.xlocationTable(0, i) = cd.xlocationTable(1, i) =  rightContactLocWorld;
        cd.zlocationTable(0, i) = cd.zlocationTable(1, i) =  rightContactLocWorldHeight;
    }
    for (int i = l_flight_hor; i < l_flight_hor + l_contact_hor; i++){
        cd.contactTable(2, i) = cd.contactTable(3, i) =  1;
        cd.xlocationTable(2, i) = cd.xlocationTable(3, i) =  leftContactLocWorld;
        cd.zlocationTable(2, i) = cd.zlocationTable(3, i) =  leftContactLocWorldHeight;
    }

    // std::cout<<"cd.xlocationTable: \n"<<cd.xlocationTable<<std::endl;
    return cd;

}

void gaitManagerRL::computeSwingTrajectoryBezier(float phase, float swingTime, int i) {

  _p[i] = Interpolate::cubicBezier<Vec3<float>>(_p0[i], _pf[i], phase);
  _v[i] = Interpolate::cubicBezierFirstDerivative<Vec3<float>>(_p0[i], _pf[i], phase) / swingTime;
  _a[i] = Interpolate::cubicBezierSecondDerivative<Vec3<float>>(_p0[i], _pf[i], phase) / (swingTime * swingTime);
  float zp, zv, za;
//   std::cout<<"phase: "<<phase<<std::endl;
  if(phase < float(0.5)) {
    zp = Interpolate::cubicBezier<float>(_p0[i][2], _p0[i][2] + _height[i], phase * 2);
    zv = Interpolate::cubicBezierFirstDerivative<float>(_p0[i][2], _p0[i][2] + _height[i], phase * 2) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<float>(_p0[i][2], _p0[i][2] + _height[i], phase * 2) * 4 / (swingTime * swingTime);
  } else {
    zp = Interpolate::cubicBezier<float>(_p0[i][2] + _height[i], _pf[i][2], phase * 2 - 1);
    zv = Interpolate::cubicBezierFirstDerivative<float>(_p0[i][2] + _height[i], _pf[i][2], phase * 2 - 1) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<float>(_p0[i][2] + _height[i], _pf[i][2], phase * 2 - 1) * 4 / (swingTime * swingTime);
  }
  _p[i][2] = zp;
  _v[i][2] = zv;
  _a[i][2] = za;

}

float gaitManagerRL::get_contact_height(float x_loc){
    float x = (x_loc<0)?0:x_loc;
    int idx = (int)floor(x / 0.05);
    return (float)terrain[idx];
}

