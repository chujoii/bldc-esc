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
	return error * proportional;
}



int apply_pid()
{
	// fixme maybe array better for coefficients than variable?
	
	int result;
	switch (g_main_ctrl_parameter){
	case 's': // velocity
		result = pid_regulator(g_velocity_ctrl - get_rpm(), g_velocity_ctrl_proportional, g_velocity_ctrl_integral, g_velocity_ctrl_derivative);
		//Serial.print("velocity ="); Serial.println(result);
		break;
	case 'u': // voltage
		result = pid_regulator(g_voltage_ctrl - g_old_ctrl_value, g_voltage_ctrl_proportional, g_voltage_ctrl_integral, g_voltage_ctrl_derivative);
		//Serial.print("voltage ="); Serial.println(result);
		break;
	case 'i': // current
		result = pid_regulator(g_current_ctrl - read_abc_current(), g_current_ctrl_proportional, g_current_ctrl_integral, g_current_ctrl_derivative);
		//Serial.print("current ="); Serial.println(result);
		break;
	default:
		result = g_old_ctrl_value;
	}
	
	if (read_abc_current() > g_limit_current_ctrl){ // fixme abs(read_abc_current()) ?
		// limit exceeded
		result = g_old_ctrl_value + pid_regulator(g_limit_current_ctrl - read_abc_current(), 1.0, 0.0, 0.0);
		Serial.println("limit: current");
	}


	
	g_old_ctrl_value = result;
	return constrain(result, pwm_min, hard_limit_voltage);

}
