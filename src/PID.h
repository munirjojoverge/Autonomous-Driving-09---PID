#ifndef PID_H
#define PID_H
#include <vector>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prev_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Saturation Control (Max and Min outputs allowed)
  */
  double Min;
  double Max;

  /*
  * Previous CPU clock time
  */
  time_t prev_time;

  /*
   When we run the PID for the very first time we don't have a dt yet and we need to get the first sys time.
  */
  bool InitializePID_time; 
  
  /*
  * This is basically a boolean to Use or Not Twiddle Algorithm.
  * If True, then twiddle will be run and the PID gains will be auto tunned.
  * The ideal case is to auto tune the PID and then simply initialize the PID with the
  * best gains and not run AutoTune anymore.
  */
  bool AutoTune;

  /*
  * Some other Twiddle variables
  */
  std::vector<double> dp;
  double dp_gain; // Diffential step increment/decrement gain: dp = dp*( 1 +- dp_gain)
  std::vector<double> TuneParams; // These are actually the Kp, Ki and Kd that will be returned
  std::vector<double> LastBestTuneParams;
  int tuning_step;
  // number of steps to allow changes to settle, then to evaluate error
  int n_settle_steps, n_eval_steps;
  double accum_quad_error, best_error;
  double tuning_tolerance; // Tuning Tolerance
  std::vector<bool> prev_incremented; // Boolean vector that will tell us in what direction we are trying to tune that parameter:1 = incremeted 
  std::vector<bool> paramsLocked; // Boolean vector that will tell us which pararemters NOT to tune. For Ex: I on the Throttle needs to stay locked at 0 
  int param_index;
  

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
  void Init(double Kp, double Ki, double Kd, double max, double min, bool AutoTune, std::vector<bool> paramsLocked);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate the total PID output given the CTE (Cross Track Error) and the dt.
  */
  double Run(double cte, double max, double min);

  /*
  * AutoTune PID parameters: Twiddle Algorithm
  */
  void TunePID(double cte);
};

#endif /* PID_H */
