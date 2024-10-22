#include "BipedGait.h"

// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string &name):
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nSegment)
{

  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * _num_cp];

  _iteration = 0;

  _offsetsFloat = offsets.cast<float>() / (float) nSegment;
  _durationsFloat = durations.cast<float>() / (float) nSegment;

  _stance = durations[0];           // number of mpc segments in stance phase
  _swing = nSegment - durations[0]; // number of mpc segments in swing phase
}

OffsetDurationGait::~OffsetDurationGait() {
  delete[] _mpc_table;
}

Vec4<float> OffsetDurationGait::getContactState() {
  Eigen::Array<float, 4, 1> progress = _phase - _offsetsFloat;

  for(int i(0); i < _num_cp; ++i)
  {
    // a bit hard to understand, but basically think the other way around  duration in the front = durationsFloat + offsetsFloat - 1
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] >= _durationsFloat[i])   // i think it should be >=
    {
      progress[i] = -1.;
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

Vec4<float> OffsetDurationGait::getSwingState()
{
  Eigen::Array<float, 4, 1> swing_offset = _offsetsFloat + _durationsFloat;
  for(int i = 0; i < _num_cp; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  Eigen::Array<float, 4, 1> swing_duration = 1. - _durationsFloat;

  Eigen::Array<float, 4, 1> progress = _phase - swing_offset;

  for(int i = 0; i < _num_cp; i++)
  {
    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] >= swing_duration[i])    // i think it should be >=
    {
      progress[i] = -1.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

int* OffsetDurationGait::getMpcTable()
{

  // printf("MPC table:\n");
  for(int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration) % _nIterations; // not sure why here it is used to be i + _iteration + 1
    Eigen::Array<int, 4, 1>  progress = iter - _offsets;
    for(int j(0); j < _num_cp; ++j)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])
        _mpc_table[i*_num_cp + j] = 1;
      else
        _mpc_table[i*_num_cp + j] = 0;

      // printf("%d ", _mpc_table[i*_num_cp + j]);
    }
    // printf("\n");
  }

  return _mpc_table;
}


void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  //Simon: _iteration find the where we are in the gait cycle
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  //Simon: _phase shows how much we have completed in the gait cycle, it's a percentage 
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);
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

void OffsetDurationGait::debugPrint() {}
