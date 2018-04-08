#include "PID.h"
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_) 
{
	Kp = Kp_;
	// Ki = Ki_;
	Kd = Kd_;
	//Initializing the parameters vector.
	params.push_back(Kp);
	// params.push_back(Ki);
	params.push_back(Kd);

	//Intiializing the delta parameters vector.
	dp = vector<double>(2);
	fill(dp.begin(), dp.end(), 0.2);
}

void PID::UpdateError()
{
	p_error = -params[0] *  cte_t;
	d_error = -params[1] * (cte_t - cte_previous)/1.0;
	// cte_sum+=cte_t;
	// i_error = -params[1] * cte_sum;
	cte_previous = cte_t;
}

double PID::TotalError() 
{
	double steer_angle = p_error + d_error; 
	return steer_angle;
}

void PID::set_drift(double drift)
{
	drift_angle = drift * M_PI/180.0;
}

void PID::param_update(int idx)
{
	params[idx]+=dp[idx];
}