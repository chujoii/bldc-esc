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


float pid_regulator(float error, float proportional, float integral, float derivative, unsigned long now_t)
{

	float sum;
	float diff; // (new_val - old_val) / (new_time - old_time)
	
	switch (g_main_ctrl_parameter){
	case 's': // velocity
		sum = error + ctrlarray[CTRL_VELOCITY].integral_accumulator;
		diff = (error - ctrlarray[CTRL_VELOCITY].old_value) / (now_t - ctrlarray[CTRL_VELOCITY].derivative_old_time);
		ctrlarray[CTRL_VELOCITY].old_value = error;
		ctrlarray[CTRL_VELOCITY].derivative_old_time = now_t;
		ctrlarray[CTRL_VELOCITY].integral_accumulator = sum;
		break;
	case 'u': // voltage
		sum = 0.0; // fixme sum
		diff = 0.0;
		break;
	case 'i': // current
		sum = error + ctrlarray[CTRL_CURRENT].integral_accumulator;
		diff = (error - ctrlarray[CTRL_CURRENT].old_value) / (now_t - ctrlarray[CTRL_CURRENT].derivative_old_time);
		ctrlarray[CTRL_CURRENT].old_value = error;
		ctrlarray[CTRL_CURRENT].derivative_old_time = now_t;
		ctrlarray[CTRL_CURRENT].integral_accumulator = sum;
	break;
	default:
		sum = 0.0;
		diff = 0.0;
	}

	return  proportional * error + integral * sum + derivative * diff;
}



int apply_pid(unsigned long halfturn_timer_us, unsigned long old_turn_timer_us, unsigned long turn_timer_us, int abc_current, unsigned long now_t)
{
	// fixme maybe array better for coefficients than variable?
	
	int result;
	
	
	
	if (abc_current > ctrlarray[CTRL_CURRENT].limit){ // fixme abs(abc_current) ?
		// limit exceeded
		result = g_old_ctrl_value + pid_regulator(ctrlarray[CTRL_CURRENT].limit - abc_current, 1.0, 0.0, 0.0, now_t); // fixme magic number
		//Serial.print("\tlimit: current "); 
	} else {
		switch (g_main_ctrl_parameter){
		case 's': // velocity
			result = pid_regulator(ctrlarray[CTRL_VELOCITY].value - get_rpm(halfturn_timer_us, old_turn_timer_us, turn_timer_us), ctrlarray[CTRL_VELOCITY].coeff_proportional, ctrlarray[CTRL_VELOCITY].coeff_integral, ctrlarray[CTRL_VELOCITY].coeff_derivative, now_t);
			//Serial.print("velo_ctrl = ");Serial.print(ctrlarray[CTRL_VELOCITY].value);Serial.print("\trpm = "); Serial.print(get_rpm(halfturn_timer_us, old_turn_timer_us, turn_timer_us)); Serial.print("\tveloctrl-rpm = ");Serial.print(ctrlarray[CTRL_VELOCITY].value - get_rpm(halfturn_timer_us, old_turn_timer_us, turn_timer_us));
			break;
		case 'u': // voltage
			result = pid_regulator(ctrlarray[CTRL_VOLTAGE].value - g_old_ctrl_value, ctrlarray[CTRL_VOLTAGE].coeff_proportional, ctrlarray[CTRL_VOLTAGE].coeff_integral, ctrlarray[CTRL_VOLTAGE].coeff_derivative, now_t);
			//Serial.print("voltage = ");
			break;
		case 'i': // current
			result = pid_regulator(ctrlarray[CTRL_CURRENT].value - abc_current, ctrlarray[CTRL_CURRENT].coeff_proportional, ctrlarray[CTRL_CURRENT].coeff_integral, ctrlarray[CTRL_CURRENT].coeff_derivative, now_t);
			//Serial.print("current_ctrl = ");Serial.print(ctrlarray[CTRL_CURRENT].value);Serial.print("\ti = "); Serial.print(abc_current); Serial.print("\tcurrent_ctl - i = ");Serial.print(ctrlarray[CTRL_CURRENT].value - abc_current);
			break;
		default:
			result = g_old_ctrl_value;
		}
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
