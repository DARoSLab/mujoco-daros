#include "MPC_logger.hpp"

MPC_logger::MPC_logger():
    _lcm_cart("udpm://239.255.76.67:7667?ttl=255")
{

}




void MPC_logger::send_info(MPC_result info){
    tello_MPC_info_lcmt msg;
    msg.horizon = info.x.size2()-1;
    for (int i = 0; i < msg.horizon+1; i++){
        msg.x[i] = info.x(0,i).scalar();
        msg.y[i] = info.x(1,i).scalar();
        msg.z[i] = info.x(2,i).scalar();
        msg.dx[i] = info.xd(0,i).scalar();
        msg.dy[i] = info.xd(1,i).scalar();
        msg.dz[i] = info.xd(2,i).scalar();
        msg.wx[i] = info.xw(0,i).scalar();
        msg.wx[i] = info.xw(1,i).scalar();
        msg.wx[i] = info.xw(2,i).scalar();
    }
    _lcm_cart.publish("mpcInfo", &msg);
}

