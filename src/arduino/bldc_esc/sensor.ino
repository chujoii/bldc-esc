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


byte digital_read_angle(int state_a_hall, int state_b_hall, int state_c_hall, int hall_zero)
{
	boolean b_state_a_hall, b_state_b_hall, b_state_c_hall;
	byte state_abc_hall = 0;

	b_state_a_hall = state_a_hall > hall_zero;
	b_state_b_hall = state_a_hall > hall_zero;
	b_state_c_hall = state_a_hall > hall_zero;
	
	state_abc_hall = (b_state_c_hall<<2) | (b_state_b_hall<<1) | b_state_a_hall;
	
	return state_abc_hall;
}


float analog_read_angle (float state_a_hall, float state_b_hall, float state_c_hall, float hall_min, float hall_max)
{
	float normalize_a_hall = fmap((float)state_a_hall, hall_min, hall_max, -1.0, 1.0);
	float normalize_b_hall = fmap((float)state_b_hall, hall_min, hall_max, -1.0, 1.0);
	float normalize_c_hall = fmap((float)state_c_hall, hall_min, hall_max, -1.0, 1.0);

	float angle = calculation_angle_from_three_phases(normalize_a_hall, normalize_b_hall, normalize_c_hall);

	//DEBUGA_PRINT(angle);
	//DEBUGA_PRINT("\t");

	return angle;
}



int read_abc_current()
{
	return analogRead(PIN_ANALOG_ABC_CURRENT) - g_zero_abc_current;
}



int get_rpm(unsigned long halfturn_timer_us, unsigned long old_turn_timer_us){
	unsigned long full_turn_time = g_turn_timer_us - old_turn_timer_us;
	unsigned long half_turn_time = halfturn_timer_us - old_turn_timer_us;
	return (int) (60000000 / max(half_turn_time, full_turn_time));
}


float angular_velocity()
{
	
}


void sync_sensor_measurement(int *a_hall, int *b_hall, int *c_hall)
{
	// fixme: reading sensors must be simultaneously

	*a_hall = analogRead(PIN_ANALOG_A_HALL);
	*b_hall = analogRead(PIN_ANALOG_B_HALL);
	*c_hall = analogRead(PIN_ANALOG_C_HALL);
	
	g_abc_current = analogRead(PIN_ANALOG_ABC_CURRENT) - g_zero_abc_current;
}


void sensor_statistic(int n, int lim, int a_hall_value, int b_hall_value, int c_hall_value, int *hall_min, int *hall_max, int *hall_zero)
{
	// n   - mean coefficient
	// lim - difference betwin new element, and stoded extremum, so only if new near extremum it can influence to extremum
	int a, b, c;
	int m;
	// (a * n + b) / (n + 1)       =       a + ((b - a) / (n + 1))

	
	
	
	a = *hall_max;
	b = max(max(a_hall_value, b_hall_value), c_hall_value);
	if ((a<b) || (abs(a-b)<lim)) {m=1;} else {m=n;} // if b=maximal -> element have more weigth
	c = a + ((b - a) / (m + 1));
	*hall_max = c;
		

	a = *hall_min;
	b = min(min(a_hall_value, b_hall_value), c_hall_value);
	if ((a>b) || (abs(a-b)<lim)) {m=1;} else {m=n;} // if b=minimal  -> element have more weigth
	c = a + ((b - a) / (m + 1));
	*hall_min = c;
		
	
	*hall_zero = ((*hall_max) + (*hall_min))/2;
}



