#ifndef __PRESTOE_DEFINTION_H__
#define __PRESTOE_DEFINTION_H__

#include <iostream>

namespace prestoe
{
    constexpr size_t num_act_joint = 23;
    constexpr size_t num_leg_joint = 7;
    constexpr size_t num_arm_joint = 4;
    constexpr size_t nDoF = num_act_joint + 6;

    constexpr size_t RKneeIdx = 4;
    constexpr size_t LKneeIdx = 11;
} // namespace prestoe

#endif