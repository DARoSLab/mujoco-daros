#ifndef BIPED_GAIT_H
#define BIPED_GAIT_H

#include <string>
#include <queue>

#include "cppTypes.h"


class BipedGait {
public:
  virtual ~BipedGait() = default;

  virtual Vec4<float> getContactState() = 0;
  virtual Vec4<float> getSwingState() = 0;
  virtual int* getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;
  virtual int getCurrentGaitPhase() = 0;
  virtual void debugPrint() { }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  std::string _name;
};

using Eigen::Array4f;
using Eigen::Array4i;


class OffsetDurationGait : public BipedGait {
typedef Eigen::Array<float, 8, 1> Array8f;
typedef Eigen::Array<int, 8, 1> Array8i;

public:
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~OffsetDurationGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  static constexpr int _num_cp = 4; // num of contact point  Simon, now its 4
  int* _mpc_table;
  Array4i _offsets; // offset in mpc segments (when contact is made)
  Array4i _durations; // duration of step in mpc segments
  Array4f _offsetsFloat; // offsets in phase (0 to 1)
  Array4f _durationsFloat; // durations in phase (0 to 1)
  int _stance;
  int _swing;
  int _iteration;
  int _nIterations;     // number of mpc segments in gait cycle 
  float _phase;
};

#endif //PROJECT_GAIT_H
