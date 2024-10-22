#include  <iostream>
#include <DynSim2D.hpp>
#include <PlanarBipedRobot.hpp>

int main(argc, char **argv)
{
  PlanarBipedRobot biped;
  DynSim2D sim2D;
  sim2D.addTerrain("flat.yaml");
  sim2D.addRobot(&biped);
  sim2D.addObject("ball.yaml");

  for(int i(0); i < 1000; ++i)
  {
    sim2D.step();
    if(i % 10 == 0)
    {
      sim2D.send_message();
    }
    usleep(1000);
  }
  return 0;
}
