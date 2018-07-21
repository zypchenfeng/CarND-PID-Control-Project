#ifndef PID_H
#define PID_H


#include <uWS/uWS.h>
#include <iostream>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double best_error;
  double total_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  int counter;
  int num_twiddles;
  double tuning_sign[3];
  double tuning_constant[3];
  double best_p[3];
  int twiddle_index;

  double p[3];
  double dp[3];

  bool use_twiddle = false;
  bool retry = false;

  uWS::WebSocket<uWS::SERVER> server;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void RestartSim(uWS::WebSocket<uWS::SERVER> ws);

  double Steering();

  void Twiddle();

};

#endif /* PID_H */
