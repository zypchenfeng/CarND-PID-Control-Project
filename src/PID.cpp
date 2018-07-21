#include "PID.h"
#include<limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;
	dp[0] = 0.01;
	dp[1] = 0.0001;
	dp[2] = 0.1;
	best_p[0] = Kp;
	best_p[1] = Ki;
	best_p[2] = Kd;
	p_error = numeric_limits<double>::max();
	i_error = 0;
	d_error = 0;
	counter = 0;
	num_twiddles = 0;
	use_twiddle = false;
	best_error = -1;
	total_error = 0;
	tuning_sign[0] = 1;
	tuning_sign[1] = 1;
	tuning_sign[2] = 1;
	tuning_constant[0] = 1.0;
	tuning_constant[1] = 1.0;
	tuning_constant[2] = 1.0;
	twiddle_index = 0;

}

void PID::UpdateError(double cte) {
	if (p_error == numeric_limits<double>::max())
		p_error = cte;
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;	
}

double PID::TotalError() {
	return p_error * p_error;
}

double PID::Steering() {
	double steer_value = -(Kp * p_error + Kd * d_error + Ki * i_error);
	//Make sure the steering is within [-1, 1]
	while (steer_value > 1)
		steer_value -= 2;
	while (steer_value < -1)
		steer_value += 2;
	return steer_value;
}

void PID::Twiddle()
{
	if (use_twiddle) {
		if (counter == 200) {
			total_error = TotalError();
		}
		if (counter > 500) {
			total_error += TotalError();
		}
		if (counter > 500) {
			cout << "\nStart twiddling" << endl;
			if (best_error == -1) 
				best_error = total_error + 1;
			double sum_dp = accumulate(begin(dp), end(dp), 0.0, plus<double>());
			if (twiddle_index > 2) twiddle_index = 0; // reset twiddle index
			if ((sum_dp > 0.001) & (num_twiddles < 5000)) {		
				cout << "\nCurrent twiddle index is: " << twiddle_index << endl;
				counter = 0;
				if (retry) {
					if (total_error < best_error) {
						best_error = total_error;
						best_p[0] = p[0];
						best_p[1] = p[1];
						best_p[2] = p[2];
						dp[twiddle_index] *= 1.1;
						twiddle_index += 1;
						num_twiddles += 1;
						retry = false;
					}
					else {
						p[twiddle_index] = best_p[twiddle_index];
						dp[twiddle_index] *= 0.9;
						p[twiddle_index] += dp[twiddle_index];
						num_twiddles += 1;
					}
				}
				else {
					if (total_error < best_error) {
						best_error = total_error;
						best_p[0] = p[0];
						best_p[1] = p[1];
						best_p[2] = p[2];
						dp[twiddle_index] *= 1.1;
						twiddle_index += 1;
						num_twiddles += 1;
						retry = false;
					}
					else {
						p[twiddle_index] -= 2*dp[twiddle_index];
						dp[twiddle_index] *= 0.9;
						num_twiddles += 1;
						retry = true;
					}

				}
				
				
			}
			else {
				use_twiddle = false;
				cout << "Stopping twiddling!!!!\n";
			}
		}
	}
	Kp = p[0];
	Ki = p[1];
	Kd = p[2];
	counter += 1;
}

void PID::RestartSim(uWS::WebSocket<uWS::SERVER> ws) {
	Init(p[0], p[1], p[2]);
	std::string reset_msg = "42[\"reset\",{}]";
	ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
