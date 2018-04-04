#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) 
{
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
}

void PID::UpdateError()
{
	p_error = -Kp *  cte_t;
	d_error = -Kd * (cte_t - cte_previous)/1.0;
	cte_sum+=cte_t;
	i_error = -Ki * cte_sum;
	cte_previous = cte_t;
}

double PID::TotalError() 
{
	double steer_angle = p_error + d_error + i_error + drift_angle;
	return steer_angle;
}

void PID::set_drift(double drift)
{
	drift_angle = drift * M_PI/180.0;
}

