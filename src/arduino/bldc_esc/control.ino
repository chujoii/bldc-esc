/*
  control.ino --- prepare data from sensors (hall sensors, angle of rotor, current, ...) for motor

  Copyright (C) 2016 Roman V. Prikhodchenko



  Author: Roman V. Prikhodchenko <chujoii@gmail.com>


  This file is part of bldc-esc.

    bldc-esc is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    bldc-esc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with bldc-esc.  If not, see <http://www.gnu.org/licenses/>.



 Keywords: bldc esc hall bemf current motor sensor optocouple



 Usage:


 History:

 Version 0.1 was created at 2013.may.18



 Code:
*/


int apply_pid(unsigned long halfturn_timer_us, unsigned long old_turn_timer_us, unsigned long turn_timer_us, int abc_current, unsigned long now_t, struct ctrl *ctrlarray)
{
	// fixme maybe array better for coefficients than variable?
	
	int result;
	
	float error;

	float diff; // (new_val - old_val) / (new_time - old_time)
	
	
	if (abc_current > ctrlarray[CTRL_CURRENT].limit){ // fixme abs(abc_current) ?
		// limit exceeded
		error              = ctrlarray[CTRL_CURRENT].limit - abc_current;
		float coeff_proportional = 1.0; // fixme: magic number
		// coeff_integral     = 0.0;    // cannot calculate, fixme: magic number
		// coeff_derivative   = 0.0;    // cannot calculate, fixme: magic number
		//sum = 0.0;
	        //diff = 0.0;
		result = g_old_ctrl_value + (coeff_proportional * error); // + 0
		//Serial.print("\tlimit: current "); 
	} else {
		switch (g_main_ctrl_parameter){
		case CTRL_VELOCITY:
			error = ctrlarray[CTRL_VELOCITY].value - get_rpm(halfturn_timer_us, old_turn_timer_us, turn_timer_us);
			break;
		case CTRL_VOLTAGE:
			error = ctrlarray[CTRL_VOLTAGE].value - g_old_ctrl_value;
			break;
		case CTRL_CURRENT:
			error = ctrlarray[CTRL_CURRENT].value - abc_current;
			break;
		default:
			error = 0.0;
		}

		ctrlarray[g_main_ctrl_parameter].integral_accumulator += error; // sum

		diff = (error - ctrlarray[g_main_ctrl_parameter].old_value) / (now_t - ctrlarray[g_main_ctrl_parameter].derivative_old_time);

		result = ctrlarray[CTRL_VELOCITY].coeff_proportional * error +
			ctrlarray[CTRL_VELOCITY].coeff_integral      * ctrlarray[g_main_ctrl_parameter].integral_accumulator +
			ctrlarray[CTRL_VELOCITY].coeff_derivative    * diff;
	}

	//Serial.print("\tresult = ");Serial.println(result);
	
	g_old_ctrl_value = result;
	return constrain(result, -DAC_DRIVER_MAX, DAC_DRIVER_MAX);
}



float calculate_analog_angle_shift_by_velocity(int velocity)
{
	if (velocity>0) {
		return g_analog_abc_shift_cw;
	} else {
		return g_analog_abc_shift_ccw;
	}

}
