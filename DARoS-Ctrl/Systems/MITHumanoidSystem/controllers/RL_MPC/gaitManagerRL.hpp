#ifndef GAITMANAGERRL_H
#define GAITMANAGERRL_H

#include <deque>
#include <cppTypes.h>
#include <iostream>
#include <TelloParameters.h>
#include <robots/Tello.h>
using namespace std;
using namespace Eigen;

struct ContactData
{
    Eigen::MatrixXd contactTable; 
    Eigen::MatrixXd xlocationTable;
    Eigen::MatrixXd zlocationTable;

};


class gaitManagerRL {
    public:
        gaitManagerRL(TelloParameters* parameters, int iteration_between_mpc, Eigen::VectorXd terrain);
        ~gaitManagerRL(){}

        void take2dAct(VectorXd act2d, float curr_contact_loc);
        ContactData rl2dActToContactData(VectorXd act2d, float curr_contact_loc);
        void updates();
        int getStates();
        void setIterations(int iter);
        float swingPhase[4] = {0,0,0,0};
        int inAirTime[4] = {0,0,0,0};
        bool isSwing[4] = {false, false, false, false};

        float rightSwingTime = 0;
        float swingTimes[4] = {0,0,0,0};

        void setInitialPosition(Vec3<float> p0, int i) {
            _p0[i] = p0;
        }
        void setFinalPosition(Vec3<float> pf, int i) {
            _pf[i] = pf;
        }
        void setHeight(float h, int i) {
            _height[i] = h;
        }

        void computeSwingTrajectoryBezier(float phase, float swingTime, int i);
        Vec3<float> _p0[4], _pf[4], _p[4], _v[4], _a[4];
        Eigen::MatrixXd mpcTable;
        Eigen::MatrixXd mpcLocTableX;
        Eigen::MatrixXd mpcLocTableZ;

        // the entire contact table for the current RL input, 1 for contact, 0 for flight
        /*
            111110000011111  right heel
            111110000011111  right toe
            000011111110000  left heel
            000011111110000  left toe
        */
        Eigen::MatrixXd contactTable; 
        // the location of the contact point in the world frame x axis. 
        Eigen::MatrixXd xlocationTable;
        Eigen::MatrixXd zlocationTable;

        // the table of the RL output based on the mpc prediction. 
        Eigen::MatrixXd contactTableNext;
        Eigen::MatrixXd xlocationTableNext;
        Eigen::MatrixXd zlocationTableNext;

        Eigen::MatrixXd contactTableCombined;
        Eigen::MatrixXd xlocationTableCombined;
        Eigen::MatrixXd zlocationTableCombined;

        double xlandingloc[4] = {0,0,0,0};
        double zlandingloc[4] = {0,0,0,0};
        double prezlandingloc[4] = {0,0,0,0};
        int idxTable = 999;
        float get_contact_height(float x_loc);

    private:
        TelloParameters* p = nullptr;
        int _iter = 0;
        Eigen::VectorXd terrain;
        int timeToPop;
        int _iteration_between_mpc;


        float _height[4];
        



};

#endif

