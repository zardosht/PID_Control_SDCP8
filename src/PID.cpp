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

void PID::UpdateError(double cte) {
  p_error = -Kp * cte;
  
  double delta_cte = cte - prev_cte;
  prev_cte = cte;
  d_error = -Kd * delta_cte;

  sum_cte += cte;
  i_error = -Ki * sum_cte; 
}

double PID::TotalError() {
  // return p_error + d_error + i_error;
  return p_error + d_error;

}