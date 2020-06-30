#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double error) {
  p_error = -Kp * error;
  
  sum_error += error;
  i_error = -Ki * sum_error; 
  
  double delta_error = error - prev_error;
  prev_error = error;
  d_error = -Kd * delta_error;

}

double PID::TotalError() {
  return p_error + i_error + d_error;

}