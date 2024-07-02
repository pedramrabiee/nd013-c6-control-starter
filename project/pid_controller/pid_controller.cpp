/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   Kp = Kpi;
   Ki = Kii;
   Kd = Kdi;
   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;
   error = 0.0;
   p_err = 0;
   d_err = 0;
   i_err = 0;
   dt = 0.0;
}


void PID::UpdateError(double cte) {
   error = cte;
   
   d_err = 0.0;
   if (dt > 0)
   {
      d_err = (cte - p_err) / dt;
   }   
   p_err = cte;
   i_err += cte;

}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */

   
   double control = Kp * p_err + Kd * d_err + Ki * i_err;
   control = std::min(std::max(control, output_lim_max), output_lim_min);
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   dt = new_delta_time;
}