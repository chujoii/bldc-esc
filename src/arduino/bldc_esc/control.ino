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


float pid_regulator(float error, float proportional, float integral, float derivative)
{

	float sum;
	float diff; // (now_val - old) / (now_t - oldt)
	unsigned long now_t = micros();
	
	switch (g_main_ctrl_parameter){ // fixme too many global var
	case 's': // velocity
		sum = error + g_voltage_ctrl_integral_old_val;
		diff = (error - g_voltage_ctrl_derivative_old_val) / (now_t - g_voltage_ctrl_derivative_old_t);
		g_voltage_ctrl_derivative_old_val = error;
		g_voltage_ctrl_derivative_old_t = now_t;
		break;
	case 'u': // voltage
		sum = 0.0;
		diff = 0.0;
		break;
	case 'i': // current
		sum = error + g_current_ctrl_integral_old_val;
		diff = (error - g_current_ctrl_derivative_old_val) / (now_t - g_current_ctrl_derivative_old_t);
		g_current_ctrl_derivative_old_val = error;
		g_current_ctrl_derivative_old_t = now_t;
	break;
	default:
		sum = 0.0;
		diff = 0.0;
	}

	return  proportional * error + integral * sum + derivative * diff;
}



int apply_pid(unsigned long halfturn_timer_us, unsigned long old_turn_timer_us)
{
	// fixme maybe array better for coefficients than variable?
	
	int result;
	switch (g_main_ctrl_parameter){
	case 's': // velocity
		result = pid_regulator(g_velocity_ctrl - get_rpm(halfturn_timer_us, old_turn_timer_us), g_velocity_ctrl_proportional, g_velocity_ctrl_integral, g_velocity_ctrl_derivative);
		//Serial.print("velo_ctrl = ");Serial.print(g_velocity_ctrl);Serial.print("\trpm = "); Serial.print(get_rpm(halfturn_timer_us, old_turn_timer_us)); Serial.print("\tveloctrl-rpm = ");Serial.print(g_velocity_ctrl - get_rpm(halfturn_timer_us, old_turn_timer_us));
		break;
	case 'u': // voltage
		result = pid_regulator(g_voltage_ctrl - g_old_ctrl_value, g_voltage_ctrl_proportional, g_voltage_ctrl_integral, g_voltage_ctrl_derivative);
		//Serial.print("voltage = ");
		break;
	case 'i': // current
		result = pid_regulator(g_current_ctrl - read_abc_current(), g_current_ctrl_proportional, g_current_ctrl_integral, g_current_ctrl_derivative);
		//Serial.print("current_ctrl = ");Serial.print(g_current_ctrl);Serial.print("\ti = "); Serial.print(read_abc_current()); Serial.print("\tcurrent_ctl - i = ");Serial.print(g_current_ctrl - read_abc_current());
		break;
	default:
		result = g_old_ctrl_value;
	}
	
	
	
	if (read_abc_current() > g_limit_current_ctrl){ // fixme abs(read_abc_current()) ?
		// limit exceeded
		result = g_old_ctrl_value + pid_regulator(g_limit_current_ctrl - read_abc_current(), 1.0, 0.0, 0.0); // fixme magic number
		//Serial.print("\tlimit: current "); 
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
