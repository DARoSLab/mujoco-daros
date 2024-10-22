#include "terrainAnimator.hpp"

terrainAnimator::terrainAnimator():
    _lcm_cart("udpm://239.255.76.67:7667?ttl=255")
{

}


void terrainAnimator::send_terrain(VectorXd terrain){
    terrain_lcmt msg;
    for (int i = 0; i < terrain.size(); i++){
        msg.floor[i] = terrain[i];
    }
    _lcm_cart.publish("terrain", &msg);
}