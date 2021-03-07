#include "PID.h"

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

  /** Hyperparameters for coordinate ascent alogirthm (OptimizeParameters) **/
  
  // Include delta_parameters (for each of p,i,d)
  delta_p = 1.;
  delta_i = 1.;
  delta_d = 1.;

  // Checks for each optimize run
  best_error = TotalError();
  // To track which param we're using
  NUM_PARAMS = 3;
  // 0 -> p; 1 -> i; 2 -> d; 
  current_param = 0; 
  // To recheck error update
  recheck = false;
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


double PID::OptimizeParameters(double cte, double tolerance) {
  // TODO: reverse i_error if using UpdateError to get new error
  // Look at delta_params; if sum is less than tolerance, return params
  if (delta_p + delta_i + delta_d < tolerance) {return TotalError();}
  // Look at specific param, add delata_param associated
  double* param;
  double* param_delta;
  switch (current_param)  {
    case 0:
      param = &Kp;
      param_delta = &delta_p;
      break;
    case 1:
      param = &Ki;
      param_delta = &delta_i;
      break;
    case 2:
      param = &Kd;
      param_delta = &delta_d;
      break;
  }
  // Get error after change (from prior run)
  UpdateError(cte);
  double error = cte * cte;
  // If error is less than best, use as ref & increase delat_param*1.1
  if (error < best_error) {
    best_error = error;
    *param_delta *= 1.1;
    // Set for next parameter (cycles through parameters)
    current_param = (current_param + 1) % NUM_PARAMS;
    recheck = false;
  } else { // Error was worse so either undo or if 2nd try, reduce delta param
    if (recheck) { // Round 2 of rechecking (error still bad)
      // 2nd time error is worse, lower delta param for next time adjusting error
      *param_delta *= 0.9;
      // Set for next parameter (cycles through parameters)
      current_param = (current_param + 1) % NUM_PARAMS;
      recheck = false;
    }
    else { // First time checking; undo the param change 
      *param -= 2 * (*param_delta);
      recheck = true;
    }
  }
  double steer_value = TotalError();

  /************ Next run ************/
  // Update param with delta_param (taken care of outside of function)
  switch (current_param)  {
    case 0:
      param = &Kp;
      param_delta = &delta_p;
      break;
    case 1:
      param = &Ki;
      param_delta = &delta_i;
      break;
    case 2:
      param = &Kd;
      param_delta = &delta_d;
      break;
  }
  *param += *param_delta;

  return steer_value;

}