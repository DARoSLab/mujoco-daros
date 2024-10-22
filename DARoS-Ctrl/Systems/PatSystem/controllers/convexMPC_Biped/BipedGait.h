#ifndef BIPED_GAIT_H
#define BIPED_GAIT_H

#include <string>
#include <queue>

#include "cppTypes.h"


class BipedGait {
public:
  virtual ~BipedGait() = default;

  virtual Vec2<float> getContactState() = 0;
  virtual Vec2<float> getSwingState() = 0;
  virtual int* getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;
  virtual int getCurrentGaitPhase() = 0;
  virtual void debugPrint() { }

protected:
  std::string _name;
};

class OffsetDurationGait : public BipedGait {
public:
  OffsetDurationGait(int nSegment, Vec2<int> offset, Vec2<int> durations, const std::string& name);
  ~OffsetDurationGait();
  Vec2<float> getContactState();
  Vec2<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  static constexpr int _num_cp = 2; // num of contact point
  int* _mpc_table;
  Vec2<int> _offsets; // offset in mpc segments
  Vec2<int> _durations; // duration of step in mpc segments
  Vec2<float> _offsetsFloat; // offsets in phase (0 to 1)
  Vec2<float> _durationsFloat; // durations in phase (0 to 1)
  int _stance;
  int _swing;
  int _iteration;
  int _nIterations;
  float _phase;
};
class SwingOffsetDurationGait{
public:
  SwingOffsetDurationGait(int nSegment, Vec2<int> offset, Vec2<int> durations, const std::string& name);
  ~SwingOffsetDurationGait();
  Vec2<float> getContactState();
  Vec2<float> getSwingState();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();


private:
  static constexpr int _num_cp = 2; // num of contact point
  int* _mpc_table;
  Vec2<int> _offsets; // offset in mpc segments
  Vec2<int> _durations; // duration of step in mpc segments
  Vec2<float> _offsetsFloat; // offsets in phase (0 to 1)
  Vec2<float> _durationsFloat; // durations in phase (0 to 1)
  int _stance;
  int _swing;
  int _iteration;
  int _nIterations;
  float _phase;
};

class SimpleGait{
  public:
    SimpleGait(float swing_time, float alpha, float dt);
    void updateGait();
    Vec2<float> getContactState();
    Vec2<float> getSwingState();
    float getCurrentSwingTime(int leg);
    float getRemainingSwingTime(int leg);
    float getDoubleStanceTime();
    float getPhase(){return _phase;}
    ~SimpleGait();
  private:
    float _swing_time;
    float _alpha; //double stance to swing ratio
    float _dt;
    float _phase = 0.0;
    float _gait_period;
    Vec2<float> _contact_state;
    Vec2<float> _swing_state;


};

#endif //PROJECT_GAIT_H
