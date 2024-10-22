#ifndef VELOCITY_REVERSAL_PLANNER_LINEAR_INVERTED_PENDULUM
#define VELOCITY_REVERSAL_PLANNER_LINEAR_INVERTED_PENDULUM

#include <vector>
#include "cppTypes.h"

class ParamReversalPL{
public:
  double swing_time;
  Vec2<float> des_loc;
  Vec3<float> stance_foot_loc;
  bool b_positive_sidestep;
};

class OutputReversalPL{
public:
  double time_modification;
  double switching_state[4];
};

class Reversal_LIPM_Planner{
public:
  Reversal_LIPM_Planner();
  virtual ~Reversal_LIPM_Planner();

  virtual void PlannerInitialization(const std::string & setting_file);

  virtual void getNextFootLocation(const Vec3<float> & com_pos,
                                   const Vec3<float> & com_vel,
                                   Vec3<float> & target_loc,
                                   const void* additional_input = NULL,
                                   void* additional_output = NULL);

  // Set Functions
  void setOmega(double com_height){
    b_set_omega_ = true;
    omega_ = sqrt(9.81/com_height);
  }
  void getDesiredCOMState(
    double t,
    const Vec3<float> & com_pos_ini,  const Vec3<float> & com_vel_ini,
    const Vec3<float> & stance_foot_loc_ini,
    Vec2<float> & com_pos, Vec2<float> & com_vel);
  void CheckEigenValues(double swing_time);

protected:
  // current com state: (x, y, xdot, ydot) : 4
  // switching com state: (x, y, xdot, ydot) : 4
  // target foot: (x, y) : 2
  // swing time: t : 1
  Eigen::VectorXd planner_save_data_;

  std::vector<double> t_prime_;
  std::vector<double> kappa_;
  std::vector<double> x_step_length_limit_;
  std::vector<double> y_step_length_limit_;
  std::vector<double> com_vel_limit_;
  std::vector<double> A_;
  std::vector<double> B_;
  std::vector<double> K_;




  double omega_;
  bool b_set_omega_;

  void _computeSwitchingState(double swing_time,
                              const Vec3<float>& com_pos,
                              const Vec3<float>& com_vel,
                              const Vec3<float>& stance_foot_loc,
                              std::vector<Vec2<float>> & switching_state);
  void _StepLengthCheck(Vec3<float> & target_loc,
                        const std::vector<Vec2<float>> & switching_state);
  void _StepLengthCheck(Vec3<float> & target_loc,
                        bool b_positive_sidestep,
                        const Vec3<float> & stance_foot);

  int _check_switch_velocity(const std::vector<Vec2<float>> & switch_state);

};

#endif
