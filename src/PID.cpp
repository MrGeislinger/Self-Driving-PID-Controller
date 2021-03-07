#include "PID.h"

// TODO: Use hill climbing algorithm coordinate ascent (twiddle)

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // Initialize coeffiecents (taus)
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  // Initialize p, i, d error
	p_error = 0;
	i_error = 0;
	d_error = 0;
}

void PID::UpdateError(double cte) {
  // Proportional error
	p_error = cte;
  // Integral error: Sum of all errors
	i_error += cte;
  // Differential error: d(CTE)/dt = (cte-cte_prev)/(time_elapsed)
  d_error = cte - p_error;

}

double PID::TotalError() {
  // Total error is from proportional, differential, and integral parts
  return (Kp * p_error) + (Kd * d_error) + (Ki * i_error);
}