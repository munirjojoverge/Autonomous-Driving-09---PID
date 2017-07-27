/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: July 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#include "PID.h"
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <numeric>
#include <iostream>
#include <fstream>
#include <limits>
#include <math.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double max, double min, bool AutoTune, std::vector<bool> paramsLocked) {

	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	PID::Min = min;
	PID::Max = max;

	p_error = d_error = i_error = prev_error = 0.0;

	PID::InitializePID_time = true;

	// Twiddling parameters
	PID::AutoTune = AutoTune;

	PID::TuneParams = { Kp,Ki,Kd };
	double dpIni = 0.01;
	PID::dp = { dpIni, dpIni, dpIni };
	PID::dp_gain = 0.05;
	tuning_step = 1;	
	n_settle_steps = 100;
	n_eval_steps = 3500;
	accum_quad_error = 0;
	best_error = std::numeric_limits<double>::max();
	tuning_tolerance = 0.024;
	param_index = 0; 
	prev_incremented = { false, false, false };
	PID::paramsLocked = paramsLocked;
	if (PID::paramsLocked[0] == true && PID::paramsLocked[1] == true && PID::paramsLocked[2] == true) {
		std::cerr << "We can't AutoTune the PID with all 3 parameters locked" << std::endl;
		PID::AutoTune = false;
	}
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {	
	return p_error + i_error + d_error;
}

double PID::Run(double cte, double max, double min) {
	// If we are AutoTuning, we first produce the set of Params Kp, Ki and Kd and later we run the PID to generate
	// an output.
	// This would be just one iteration on the loop. 
	// Everytime the main program calls "Run" after geting the CTE from the sim we will get to the next iteration
	if (PID::AutoTune)
	{
		TunePID(cte); 
	}

	double output;
	double Pout;
	double Iout;
	double Dout;

	// Proportional term (independent of dt) 
	PID::p_error = cte;
	Pout = Kp * p_error;

	// If this is the first time we run the PID we don't have yet the "dt", therefore we have only a P term
	if (PID::InitializePID_time == true) {
		//std::cout << "Initializing PID time" << std::endl;
		PID::prev_time = clock();
		//std::cout << "Initial ticks " << PID::prev_time << std::endl;
		PID::prev_error = cte;
		Iout = Dout = 0.0;
		PID::InitializePID_time = false;
	}
	else
	{
		// dt
		clock_t now = clock();
		//std::cout << "Actual ticks " << now << std::endl;
		float dt = float(now - PID::prev_time) / CLOCKS_PER_SEC;
		//std::cout << "dt: " << dt << std::endl;
		PID::prev_time = now;

		// Override, since dt always comes up as 0.0 :-( The clock() function doesn't seem to work as expected for milisecods
		dt = 0.01;

		if (fabs(dt) > 0.00001) {
			// Integral term
			i_error += cte * dt;
			Iout = Ki * i_error;

			// Derivative term
			d_error = (cte - prev_error) / dt;
			Dout = Kd * d_error;
		}
		else
		{
			Iout = Dout = 0.0;
		}
	}
	

	// Calculate total output
	output = Pout + Iout + Dout;
	////std::cout << "Pout: " << Pout << " Iout: " << Iout << " Dout: " << Dout << std::endl;
	

	// Restrict to max/min (Saturation Control)
	// The Min and Max are dynamic and can change in any cycle (On the main program we will generate them)
	PID::Min = min;
	PID::Max = max;

	if (output > Max)
		output = Max;
	else if (output < Min)
		output = Min;

	// Save error to previous error
	prev_error = cte;

	return output;
	//std::cout << "PID Out: " << output << std::endl;

}

void PID::TunePID(double cte) {		
	
	if (cte > 6) { // we left the road!!
		ofstream myfile;
		myfile.open("TwiddleReport.txt");
		myfile << "We CRASHED" << endl;
		myfile << "LAST BEST PARAMS: Kp " << LastBestTuneParams[0] << " " << "Ki " << LastBestTuneParams[1] << " " << "Kd " << LastBestTuneParams[2] << endl;
		myfile << "ACTUAL PARAMS   : Kp " << PID::Kp << " " << "Ki " << PID::Ki << " " << "Kd " << PID::Kd << endl;
		myfile.close();
		AutoTune = false;
		return;
	}
	//double sum_dp = std::accumulate(PID::dp.begin(), PID::dp.end(), 0);
	double sum_dp = PID::dp[0] + PID::dp[1] + PID::dp[2];
	std::cout << "Sum dp: " << sum_dp << std::endl;
	if (sum_dp > PID::tuning_tolerance) {
		
		// Calculate the accumulated quadratic error only after n_settle_steps and before the evaluation period is over
		if (PID::tuning_step >= PID::n_settle_steps && PID::tuning_step < PID::n_eval_steps) {
			accum_quad_error += (cte*cte);
			cout << "Tuning PID Step: " << tuning_step << endl;
			cout << "Testing PID: Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << endl;
			//cout << "LAST BEST PARAMS: Kp " << LastBestTuneParams[0] << " " << "Ki " << LastBestTuneParams[1] << " " << "Kd " << LastBestTuneParams[2] << " " << endl;
			//cout << "CTE: " << cte << endl;
		}

		if (PID::tuning_step >= PID::n_eval_steps) { // We are ready to evaluate the PID parameter respose
			double norm_accum_quad_error = accum_quad_error / n_eval_steps;
			
			if (norm_accum_quad_error < best_error) { // I'm going inside here at Step 0 and ncreasing the dp without actually testing the first dp value.
				LastBestTuneParams = TuneParams;
				cout << "improvement!" << endl;
				
				best_error = norm_accum_quad_error;
				// Since we have an improvement, let's update the dp for a bigger step
				PID::dp[param_index] *= (1 + PID::dp_gain);

				// Now that we got succes with this parameter "param_index", let's move to the next one
				param_index = (param_index + 1) % 3;
				while (PID::paramsLocked[param_index] == true) {
					param_index = (param_index + 1) % 3;
				}
				// Everytime we switch to a new parameter ( Kp, Ki, Kd) we start "moving" on one direction first (+)
				PID::TuneParams[param_index] += PID::dp[param_index];
				// off we go to the next round on the simulator
			}
			else // No improvement with the direction we took or with the step (dp) we took
			{
				cout << "No improvement." << endl;
				// Since we have NO improvement, let's CHANGE direction
				if (prev_incremented[param_index] == true) {
					cout << "No improvement. Going the other direction" << endl;
					PID::TuneParams[param_index] -= 2 * PID::dp[param_index];
					prev_incremented[param_index] == false;
					// off we go to the next round on the simulator
				}
				else // We already tried both directions, therefore the dp value must be too big 
				{ 
					cout << "No improvement. Reducing dp Step" << endl;
					// first Go back to where we started on this parameter unsuccesfull step
					PID::TuneParams[param_index] += PID::dp[param_index];
					// Update the dp to make the step smaller
					PID::dp[param_index] *= (1 - PID::dp_gain);
					// we switch direction again to start all over with a smaller step
					PID::TuneParams[param_index] += PID::dp[param_index];
					prev_incremented[param_index] == true;
					// off we go to the next round on the simulator
				}

			}
			PID::Kp = PID::TuneParams[0];
			PID::Ki = PID::TuneParams[1];
			PID::Kd = PID::TuneParams[2];
			cout << "New Tuning PID: Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << endl;
			cout << "LAST BEST PARAMS: Kp " << LastBestTuneParams[0] << " " << "Ki " << LastBestTuneParams[1] << " " << "Kd " << LastBestTuneParams[2] << " " << endl;
			// once we change parameter index or change the parameter value to check the new behaviour we initialize the
			// quadradic error.
			accum_quad_error = 0;
			tuning_step = 0;
		}
		
		tuning_step += 1;
	}
	else
	{
		ofstream myfile;
		myfile.open("TwiddleReport.txt");
		myfile << "We are done Tuning for tolerance: " << tuning_tolerance << endl;
		myfile << "LAST BEST PARAMS: Kp " << LastBestTuneParams[0] << " " << "Ki " << LastBestTuneParams[1] << " " << "Kd " << LastBestTuneParams[2] << endl;
		myfile << "ACTUAL PARAMS   : Kp " << PID::Kp << " " << "Ki " << PID::Ki << " " << "Kd " << PID::Kd << endl;
		myfile.close();
		AutoTune = false;
	}
	
}

