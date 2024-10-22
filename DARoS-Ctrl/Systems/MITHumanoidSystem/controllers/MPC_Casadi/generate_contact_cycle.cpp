#include "generate_contact_cycle.hpp"
#include <iostream>
using namespace std;
Eigen::MatrixXd generate_contact_cycle(TelloParameters* p, int i){
    int total_len = p->CAS_pred_hor;
    int right_lift_idx = 6;
    int left_lift_idx = 1;
    int air_len = 4;
    Eigen::MatrixXd contact_cycle = Eigen::MatrixXd::Ones(4, total_len);

    
    // output contact cycle
    Eigen::MatrixXd selected_contact(4, total_len);

    contact_cycle.block(0, right_lift_idx,2   , air_len) = Eigen::MatrixXd::Zero(2,  air_len);
    contact_cycle.block(2, left_lift_idx, 2, air_len) = Eigen::MatrixXd::Zero(2,  air_len);


    i = i % total_len;

    int start_idx = i;
    int end_idx = i + total_len-1;
    if (end_idx >= total_len){
        int nextlen = end_idx - total_len + 1;
        Eigen::MatrixXd  frist_half  = contact_cycle.block(0, start_idx, 4, total_len - start_idx);
        Eigen::MatrixXd  second_half = contact_cycle.block(0, 0, 4, nextlen);
        selected_contact << frist_half, second_half;
    } else {
        selected_contact << contact_cycle.block(0, start_idx, 4, total_len);
    }

    return selected_contact;
}

