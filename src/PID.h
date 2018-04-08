#ifndef PID_H
#define PID_H
#include <cmath>
#include <vector>
using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  //Other Variables
  double cte_t = 0.0;
  double cte_previous = 0.0;
  double cte_sum = 0.0;
  double drift_angle = 0.0;
  vector<double> params;
  vector<double> dp;
  double error = 0.0;
  double best_error = 1000000;

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
  void Init(double Kp, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError();

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  //Set drift angle
  void set_drift(double drift);

  //To update the paramters.
  void param_update(int idx);
};

#endif /* PID_H */
