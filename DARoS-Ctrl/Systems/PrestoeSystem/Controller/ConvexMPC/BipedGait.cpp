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

  _gait_period = 2*(1 + _alpha)*_swing_time;
  _phase = 0.0;

}
SimpleGait::SimpleGait(float swing_time, float alpha, float beta, float dt):
_swing_time(swing_time),
_alpha(alpha),
_beta(beta),
_dt(dt){

  _gait_period = 2*(1 + _alpha)*_swing_time;
  _phase = 0.0;

}
void SimpleGait::updateGait(){ //ds-> right swing->ds->leftswing
  _phase = fmod(_phase + _dt, _gait_period);
  float t_stance = _gait_period - _swing_time;
  for(int leg(0); leg<2; ++leg){
    _swing_state[leg] = _phase < _swing_offset[leg] || _phase > _swing_offset[leg] + _swing_time? 0.0 : (_phase-_swing_offset[leg])/_swing_time;
    _contact_state[leg] = _swing_state[leg] > 0.0 ? 0.0 : (_phase-(_swing_offset[leg]+_swing_time))/t_stance;
    _contact_state[leg] = _contact_state[leg] < 0? _contact_state[leg] + _gait_period/t_stance: _contact_state[leg];
  }
  _updateFSM();
}
void SimpleGait::_updateFSM(){
  float t = _phase/_swing_time;
  if(t < (_alpha-_beta)){
    _fsm_state = tvr_fsm_state::DS1;
    _fsm_phase = t/(_alpha-_beta);
  }
  else if(t<_alpha){
    _fsm_state = tvr_fsm_state::T1; //T1
    _fsm_phase = (t-(_alpha-_beta))/_beta;
  }
  else if(t<_alpha-_beta + 1){
      _fsm_state = tvr_fsm_state::RS;
      _fsm_phase = (t-_alpha)/(1-_beta);
  }
  else if(t<_alpha+1){

    _fsm_state = tvr_fsm_state::T2; //T2
    _fsm_phase = (t-(_alpha-_beta + 1))/(_beta);

  }
  else if(t<(2*_alpha-_beta+1)){

    _fsm_state = tvr_fsm_state::DS2; //DS2
    _fsm_phase = (t-(_alpha + 1))/(_alpha-_beta);

  }
  else if(t<(2*_alpha+1)){

    _fsm_state = tvr_fsm_state::T3; //T3
    _fsm_phase = (t-(2*_alpha-_beta+1))/(_beta);

  }
  else if(t<(2*_alpha+2-_beta)){

    _fsm_state = tvr_fsm_state::LS; //LS
    _fsm_phase = (t-(2*_alpha+1))/(1-_beta);

  }
  else{

    _fsm_state = tvr_fsm_state::T4; //T4
    _fsm_phase = (t-(2*_alpha+2-_beta))/(_beta);

  }
  // if(_fsm_state==tvr_fsm_state::T4){
  //   std::cout << "_phase: " << _phase << '\n';
  //   std::cout << "_alpha*TS: " << _alpha*_swing_time << '\n';
  //   std::cout << "fsm_phase: " << _fsm_phase << '\n';
  //   std::cout << "================================" << '\n';
  //
  // }
}
Vec2<float> SimpleGait::getContactState(){
  return _contact_state;
}
Vec2<float>SimpleGait::getSwingState(){
  return _swing_state;
}
float SimpleGait::getCurrentSwingTime(int leg){
  return _contact_state[leg]>0.0? 0: _swing_state[leg]*_swing_time;
}
float SimpleGait::getRemainingSwingTime(int leg){
  return _contact_state[leg]>0.0? 0: (1-_swing_state[leg])*_swing_time;
}
float SimpleGait::getDoubleStanceTime(){
  return _alpha*_swing_time;
}
void OffsetDurationGait::debugPrint() {}
