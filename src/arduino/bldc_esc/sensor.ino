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


/*
void analog_hall_level_detect() // need rotate rotor by hand ~10 sec
{
	int startt = millis();
	
	// current state of hall sensor
	int state_a_hall, state_b_hall, state_c_hall;
	
	state_a_hall = analogRead(analog_pin_a_hall);
	state_b_hall = analogRead(analog_pin_b_hall);
	state_c_hall = analogRead(analog_pin_c_hall);

	a_hall_max = state_a_hall;
	a_hall_min = state_a_hall;
	b_hall_max = state_b_hall;
	b_hall_min = state_b_hall;
	c_hall_max = state_c_hall;
	c_hall_min = state_c_hall;
	
	while ((millis() - startt) < 10000){
		state_a_hall = analogRead(analog_pin_a_hall);
		state_b_hall = analogRead(analog_pin_b_hall);
		state_c_hall = analogRead(analog_pin_c_hall);
		if (state_a_hall < a_hall_min) {a_hall_min = state_a_hall;} else {if (state_a_hall > a_hall_max) {a_hall_max = state_a_hall;}}
		if (state_b_hall < b_hall_min) {b_hall_min = state_b_hall;} else {if (state_b_hall > b_hall_max) {b_hall_max = state_b_hall;}}
		if (state_c_hall < c_hall_min) {c_hall_min = state_c_hall;} else {if (state_c_hall > c_hall_max) {c_hall_max = state_c_hall;}}
	}
	
	a_hall_zero = (a_hall_max - a_hall_min)>>1; // /2
	b_hall_zero = (b_hall_max - b_hall_min)>>1; // /2
	c_hall_zero = (c_hall_max - c_hall_min)>>1; // /2
	
}
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

	state_a_hall = digital_read_hall_sensor(analog_pin_a_hall);
	state_b_hall = digital_read_hall_sensor(analog_pin_b_hall);
	state_c_hall = digital_read_hall_sensor(analog_pin_c_hall);
	
	state_abc_hall = (state_c_hall<<2) | (state_b_hall<<1) | state_a_hall;
	
	return state_abc_hall;
}


int analog_read_hall_sensor(int pin_hall)
{
	int hall_val = analogRead(pin_hall);
	if (g_hall_max < hall_val) {g_hall_max = hall_val;}
	if (g_hall_min > hall_val) {g_hall_min = hall_val;}

	return hall_val;
}


float analog_read_angle ()
{
	// current state of hall sensor
	float state_a_hall, state_b_hall, state_c_hall;
	

	state_a_hall = analog_read_hall_sensor(analog_pin_a_hall);
	state_b_hall = analog_read_hall_sensor(analog_pin_b_hall);
	state_c_hall = analog_read_hall_sensor(analog_pin_c_hall);

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
