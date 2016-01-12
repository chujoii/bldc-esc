/*
  sensor.ino --- get data from the sensors

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


boolean read_optic_sensor (int pin_sensor_output, int pin_sensor_input, int sensor_delay, int sensor_threshold_level)
{
	int dark_current = analogRead(pin_sensor_input);
	digitalWrite(pin_sensor_output, HIGH);
	delayMicroseconds(sensor_delay);
	int light_current = analogRead(pin_sensor_input);
	digitalWrite(pin_sensor_output, LOW);
	return ((light_current - dark_current) > sensor_threshold_level);
}


boolean digital_read_hall_sensor(int pin_hall)
{
	return analog_read_hall_sensor(pin_hall) > (g_hall_max + g_hall_min)/2; // g_hall_zero
}



byte digital_read_angle()
{
	// current state of hall sensor
	boolean state_a_hall, state_b_hall, state_c_hall;
	byte state_abc_hall = 0;

	state_a_hall = digital_read_hall_sensor(g_analog_pin_a_hall);
	state_b_hall = digital_read_hall_sensor(g_analog_pin_b_hall);
	state_c_hall = digital_read_hall_sensor(g_analog_pin_c_hall);
	
	state_abc_hall = (state_c_hall<<2) | (state_b_hall<<1) | state_a_hall;
	
	return state_abc_hall;
}


int analog_read_hall_sensor(int pin_hall)
{
	// switch not work: ‘g_analog_pin_a_hall’ cannot appear in a constant-expression
	if (pin_hall == g_analog_pin_a_hall){
		return g_a_hall_value;
	}
	if (pin_hall == g_analog_pin_b_hall){
		return g_b_hall_value;
	}
	// if (pin_hall == g_analog_pin_c_hall){
	return g_c_hall_value;
}


float analog_read_angle ()
{
	// current state of hall sensor
	float state_a_hall, state_b_hall, state_c_hall;
	
	state_a_hall = analog_read_hall_sensor(g_analog_pin_a_hall);
	state_b_hall = analog_read_hall_sensor(g_analog_pin_b_hall);
	state_c_hall = analog_read_hall_sensor(g_analog_pin_c_hall);

	float normalize_a_hall = fmap((float)state_a_hall, (float)g_hall_min, (float)g_hall_max, -1.0, 1.0);
	float normalize_b_hall = fmap((float)state_b_hall, (float)g_hall_min, (float)g_hall_max, -1.0, 1.0);
	float normalize_c_hall = fmap((float)state_c_hall, (float)g_hall_min, (float)g_hall_max, -1.0, 1.0);

	float angle = calculation_angle_from_three_phases(normalize_a_hall, normalize_b_hall, normalize_c_hall);

	//DEBUGA_PRINT(angle);
	//DEBUGA_PRINT("\t");

	return angle;
}



int read_abc_current()
{
	return analogRead(analog_pin_abc_current) - g_zero_abc_current;
}



int get_rpm(){
	unsigned long full_turn_time = g_turn_timer_us - g_old_turn_timer_us;
	unsigned long half_turn_time = g_halfturn_timer_us - g_old_turn_timer_us;
	return (int) (60000000 / max(half_turn_time, full_turn_time));
}



void sync_sensor_measurement()
{
	// fixme: reading sensors must be simultaneously

	g_a_hall_value = analogRead(g_analog_pin_a_hall);
	g_b_hall_value = analogRead(g_analog_pin_b_hall);
	g_c_hall_value = analogRead(g_analog_pin_c_hall);
	
	g_abc_current = analogRead(analog_pin_abc_current) - g_zero_abc_current;
}


void sensor_statistic(int n, int lim)
{
	// n   - mean coefficient
	// lim - difference betwin new element, and stoded extremum, so only if new near extremum it can influence to extremum
	float a, b, c;
	float m;
	// (a * n + b) / (n + 1)       =       a + ((b - a) / (n + 1))

	
	
	
	a = g_hall_max;
	b = max(max(g_a_hall_value, g_b_hall_value), g_c_hall_value);
	if ((a<b) || (abs(a-b)<lim)) {m=1;} else {m=n;} // if b=maximal -> element have more weigth
	c = a + ((b - a) / (m + 1));
	g_hall_max = c;
		

	a = g_hall_min;
	b = min(min(g_a_hall_value, g_b_hall_value), g_c_hall_value);
	if ((a>b) || (abs(a-b)<lim)) {m=1;} else {m=n;} // if b=minimal  -> element have more weigth
	c = a + ((b - a) / (m + 1));
	g_hall_min = c;
		
	
	g_hall_zero = (g_hall_max + g_hall_min)/2;
}



void analog_hall_level_detect_first_run()
{
	g_hall_max = max(max(g_a_hall_value, g_b_hall_value), g_c_hall_value);
	g_hall_min = min(min(g_a_hall_value, g_b_hall_value), g_c_hall_value);
}
