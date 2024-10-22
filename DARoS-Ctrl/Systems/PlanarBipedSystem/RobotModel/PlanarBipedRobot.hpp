#ifndef _PlanarBipedRobot_H
#define _PlanarBipedRobot_H



template <typename T>
class PlanarBipedRobot
{
  public:
    PlanarBipedRobot();
    ~PlanarBipedRobot();

    void updateState(DVec<T> &q, DVec<T> &dq);
    void getMassMatrix(DMat<T> &M) { M = _massMtx; }
    void getGravity(DVec<T> &grav) { grav = _gravity; }
    void getCoriolis(DVec<T> &cor) { cor = _coriolis; }
    void getLinkJacobian(DMat<T> &J, int linkIdx) { J = _linkJacobian[linkIdx]; }
};

#endif /* ifndef _PlanarBipedRobot_H  */
