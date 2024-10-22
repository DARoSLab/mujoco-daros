#include "BipedGait.h"
#include <iostream>

// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait(int nSegment,
    Vec2<int> offsets, Vec2<int> durations, const std::string &name) :
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nSegment)
{

  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * _num_cp];

  _offsetsFloat = offsets.cast<float>() / (float) nSegment;
  _durationsFloat = durations.cast<float>() / (float) nSegment;

  _stance = durations[0];
  _swing = nSegment - durations[0];
}

OffsetDurationGait::~OffsetDurationGait() {
  delete[] _mpc_table;
}

Vec2<float> OffsetDurationGait::getContactState() {
  Vec2<float> progress;

  for(int i(0); i < _num_cp; ++i)
  {
    progress[i] = _phase - _offsetsFloat[i];

    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _durationsFloat[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  return progress.matrix();
}

Vec2<float> OffsetDurationGait::getSwingState()
{
  Vec2<float> swing_offset = _offsetsFloat + _durationsFloat;
  Vec2<float> swing_duration;
  for(int i = 0; i < _num_cp; i++){
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;

    swing_duration[i] = 1. - _durationsFloat[i];
  }

  Vec2<float> progress;

  for(int i = 0; i < _num_cp; i++)
  {
    progress[i] = _phase - swing_offset[i];

    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  return progress.matrix();
}

int* OffsetDurationGait::getMpcTable()
{

  for(int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Vec2<int> progress;

    for(int j(0); j < _num_cp; ++j)
    {
      progress[j] = iter - _offsets[j];
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])
        _mpc_table[i*_num_cp + j] = 1;
      else
        _mpc_table[i*_num_cp + j] = 0;

      //printf("%d ", _mpc_table[i*_num_cp + j]);
    }
    //printf("\n");
  }

  return _mpc_table;
}


void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations))
    / (float) (iterationsPerMPC * _nIterations);
}


int OffsetDurationGait::getCurrentGaitPhase() {
  return _iteration;
}

float OffsetDurationGait::getCurrentSwingTime(float dtMPC, int leg) {
  (void)leg;
  return dtMPC * _swing;
}

float OffsetDurationGait::getCurrentStanceTime(float dtMPC, int leg) {
  (void) leg;
  return dtMPC * _stance;
}
SimpleGait::SimpleGait(float swing_time, float alpha, float dt):
_swing_time(swing_time),
_alpha(alpha),
_dt(dt){

  _gait_period = (2 + _alpha)*_swing_time;

}
void SimpleGait::updateGait(){ //ds->left swing->right swing
  _phase = fmod(_phase + _dt, _gait_period);
  // if(_phase < _alpha*_swing_time){//double stance
  if(_phase < _swing_time){//right swing
    // _contact_state[0] = _phase/((1+_alpha)*_swing_time);
    // _contact_state[1] = _phase/((1+_alpha)*_swing_time) + 1.0/(1.0 + _alpha);

    _contact_state[0] = 0.0;
    _contact_state[1] = _phase/((1+_alpha)*_swing_time);
    _swing_state[0] = _phase/_swing_time;
    _swing_state[1] = 0.0;

  // }else if(_phase < (1+_alpha)*_swing_time){ // left swing
  }else if(_phase < (1+_alpha)*_swing_time){ // double stance

    _contact_state[0] = (_phase-_swing_time)/((1+_alpha)*_swing_time);
    _contact_state[1] = _phase/((1+_alpha)*_swing_time);
    _swing_state[0] = 0.0;
    _swing_state[1] = 0.0;

  }
  else{ //left swing

    // _contact_state[1] = (_phase - (1+_alpha)*_swing_time)/((1+_alpha)*_swing_time);
    _contact_state[0] = (_phase -_swing_time)/((1+_alpha)*_swing_time);
    _contact_state[1] = 0.0;
    _swing_state[0] = 0.0;
    _swing_state[1] = (_phase -(1+_alpha)*_swing_time)/_swing_time;

  }
}
Vec2<float> SimpleGait::getContactState(){
  return _contact_state;
}
Vec2<float>SimpleGait::getSwingState(){
  return _swing_state;
}
float SimpleGait::getCurrentSwingTime(int leg){
  if(_contact_state[leg]>0.0){
    return 0.0;
  }else{
    return _phase - ((leg + 1)%2 + _alpha)*_swing_time;
  }
}
float SimpleGait::getRemainingSwingTime(int leg){
  if(_contact_state[leg]>0.0){
    return 0.0;
  }else{
      // return ((leg + 1)%2 + _alpha + 1)*_swing_time-_phase;
      return leg==1?_gait_period - _phase: _swing_time - _phase;
  }
}
float SimpleGait::getDoubleStanceTime(){
  return _alpha*_swing_time;
}
void OffsetDurationGait::debugPrint() {}
