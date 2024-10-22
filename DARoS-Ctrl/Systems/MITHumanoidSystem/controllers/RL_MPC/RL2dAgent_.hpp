#ifndef RLAGENTt_H
#define RLAGENTt_H

#include <unistd.h>
#include <typeinfo>
#include "loco-jump/utils/State2d.hpp"
#include <Configuration.h>
class RL2dAgent_ {
    public:
    RL2dAgent_();
    ~RL2dAgent_(){}

    VectorXd module_forward(State2d &s);

    VectorXd post_process(VectorXd &action);
};

#endif

