/*
  bldc_esc.ino --- control program for bldc-esc

  Copyright (C) 2012 Roman V. Prikhodchenko



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

#include <math.h>

// pins:     [1 2] 3 4 5 6 7 8 9 10 11 12 13 A0 A1 A2 A3 A4 A5
// free pin: [1 2]                     12 13          A3 A4 A5
// used pin: [1 2] 3 4 5 6 7 8 9 10 11       A0 A1 A2 A3 A4 A5


// phase a, b, c
const int pin_a_hi = 3;
const int pin_a_lo = 5;
const int pin_b_hi = 6;
const int pin_b_lo = 9;
const int pin_c_hi = 10;
const int pin_c_lo = 11;

// global interrupt pin
int pin_global_interrupt = 2;

float angle_a_shift =   0.0;  
float angle_b_shift = 120.0;
float angle_c_shift = 240.0;

const float point_of_symmetry_sin_x_plus  =  90.0;
const float point_of_symmetry_sin_x_minus = 270.0;
const float poin_of_zero_cross_sin_x      = 180.0;

// ///////////////////////////////////////////// sensors ////////

// Hall a, b, c (or other type of sensor: for example optic)
//const int analog_pin_a_hall = A0;
//const int analog_pin_b_hall = A1;
//const int analog_pin_c_hall = A2;

//const int digital_pin_a_hall = 4;
//const int digital_pin_b_hall = 7;
//const int digital_pin_c_hall = 8;

// hall max min level
int a_hall_max;
int a_hall_min;
int a_hall_zero;

int b_hall_max;
int b_hall_min; 
int b_hall_zero;

int c_hall_max;
int c_hall_min;
int c_hall_zero;

const int analog_pin_a_optocouple = A0;
const int analog_pin_b_optocouple = A1;
const int analog_pin_c_optocouple = A2;

const int pin_a_cotrol_optocouple = 4;
const int pin_b_cotrol_optocouple = 7;
const int pin_c_cotrol_optocouple = 8;

const byte optocouple_delay = 1; // us   fixme: need experiments
int optocouple_threshold_level = 128; // fixme: need experiments


// BEMF sensors
//const int analog_pin_a_bemf = A0;
//const int analog_pin_b_bemf = A1;
//const int analog_pin_c_bemf = A2;

// Current sensors
//const int analog_pin_a_current = A3;
//const int analog_pin_b_current = A4;
//const int analog_pin_c_current = A5;
//const int analog_pin_abc_current = A3;




const byte pwm_min = 0;
const byte pwm_max = 255;

int delay_between_step = 1; // us


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

float calculation_angle_from_three_phases(float a, float b, float c) // int prev_angle, int prev_step)
{
	// all value in degree

	// a = sin(x)
	// b = sin(x+120)
	// c = sin(x+240)
	// x = ?


	// Each arc sine gives two solutions for the period.
	// for example real_motor_angle = 230 (degree)
	// and we need calculate motor_angle
	//
	// read a, b, c from sensors:
	// a ~ -0.77
	// b ~ -0.17
	// c ~ +0.92
	//
	// all value in degree
	// x[rad]  = x[grad] * (3.14/180)
        // x[grad] = x[rad]  * (180/3.14)
	//
	// angle_a_1 = (- (* (asin -0.77) (/ 180.0 3.14))   0) =  -50.4 [degree]
	// angle_b_1 = (- (* (asin -0.17) (/ 180.0 3.14)) 120) = -129.8 [degree]
        // angle_c_1 = (- (* (asin  0.92) (/ 180.0 3.14)) 240) = -173.0 [degree]
	//
	// period of sin = 360[degree] = 2*3.14[rad]
	// 
	//
	// if (angle_x_1 < 0)   {angle_x_1 = angle_x_1 + 360;}
	//
	// angle_a_1 = 309.6
	// angle_b_1 = 230.2 + 120
        // angle_c_1 = 187.0 + 240
	//
	// if (angle_x_1 < poin_of_zero_cross_sin_x) { angle_x_2 = point_of_symmetry_sin_x_plus  * 2 - angle_x_1;}
	// else                 { angle_x_2 = point_of_symmetry_sin_x_minus * 2 - angle_x_1;}
	//
	// angle_a_2 = 230.4
	// angle_b_2 = 309.8 + 120
        // angle_c_2 = 353.0 + 240
	// 
	//
	//
	//
	//
	//
	//
	//
	//



		
	// fixme need change math function asin to table function
	// fixme float -> int

	// http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
	// The asin() function computes the principal value of the arc
	// sine of __x. The returned value is in the range [-pi/2,
	// pi/2] radians. A domain error occurs for arguments not in
	// the range [-1, +1].
	float angle_a = asin(a) * (180/3.14) - angle_a_shift; // - 0
	float angle_b = asin(b) * (180/3.14) - angle_b_shift; // - 120
	float angle_c = asin(c) * (180/3.14) - angle_c_shift; // - 240
	


	
	float diff_1 = 
	
	
	return (angle_a + angle_b + angle_c)/3.0;
}

boolean unit_test_calculation_angle_from_three_phases()
{
	boolean result = true;
	if (){}
}

/*
void turn_analoghall(int diretcion) // not work
{
	// current state of hall sensor
	int state_a_hall, state_b_hall, state_c_hall;
	
	state_a_hall = analogRead(analog_pin_a_hall);
	state_b_hall = analogRead(analog_pin_b_hall);
	state_c_hall = analogRead(analog_pin_c_hall);
	
	int u_control_a_hi = map(state_a_hall, a_hall_min, a_hall_max, -pwm_max, pwm_max); // fixme: need use direction
	int u_control_a_lo = map(state_a_hall, a_hall_min, a_hall_max, -pwm_max, pwm_max); // fixme: need use direction

	int u_control_b_hi = map(state_b_hall, b_hall_min, b_hall_max, -pwm_max, pwm_max); // fixme: need use direction
	int u_control_b_lo = map(state_b_hall, b_hall_min, b_hall_max, -pwm_max, pwm_max); // fixme: need use direction

	int u_control_c_hi = map(state_c_hall, c_hall_min, c_hall_max, -pwm_max, pwm_max); // fixme: need use direction
	int u_control_c_lo = map(state_c_hall, c_hall_min, c_hall_max, -pwm_max, pwm_max); // fixme: need use direction

	//if (u_control_a<0)
	digitalWrite(pin_a_hi, abs(u_control_a_hi));
	digitalWrite(pin_a_lo, abs(u_control_a_lo));
	digitalWrite(pin_b_hi, abs(u_control_b_hi));
	digitalWrite(pin_b_lo, abs(u_control_b_lo));
	digitalWrite(pin_c_hi, abs(u_control_c_hi));
	digitalWrite(pin_c_lo, abs(u_control_c_lo));
}
*/



void turn_digital(int direction)
{

	byte speed;
	
	speed = abs(direction);


	// current state of hall sensor
	boolean state_a_hall, state_b_hall, state_c_hall;
	byte state_abc_hall = 0;


	state_a_hall = read_optic_sensor (pin_a_cotrol_optocouple, analog_pin_a_optocouple, optocouple_delay, optocouple_threshold_level);
	state_b_hall = read_optic_sensor (pin_b_cotrol_optocouple, analog_pin_b_optocouple, optocouple_delay, optocouple_threshold_level);
	state_c_hall = read_optic_sensor (pin_c_cotrol_optocouple, analog_pin_c_optocouple, optocouple_delay, optocouple_threshold_level);
	
	
	if (state_a_hall) {state_abc_hall = state_abc_hall & B100;};
	if (state_b_hall) {state_abc_hall = state_abc_hall & B010;};
	if (state_c_hall) {state_abc_hall = state_abc_hall & B001;};
	
	/*
	  Switches commutation for rotor rotation

	  ahi           bhi           chi
	  alo           blo           clo
	  

	  hall        phase     switches
	  101         a-b       a-hi b-lo
	  001         a-c       a-hi c-lo
	  011         b-c       b-hi c-lo
	  010         b-a       b-hi a-lo
	  110         c-a       c-hi a-lo
	  100         c-b       c-hi b-lo

	  111         error
	  000         error

	 */



	
	
	switch (state_abc_hall) {
	case B101:
		// a-hi b-lo
		analogWrite(pin_a_hi, speed);
		analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
		analogWrite(pin_b_hi, 0);
		analogWrite(pin_b_lo, speed); // digitalWrite(pin_b_lo, HIGH)
		analogWrite(pin_c_hi, 0);
		analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
		break;
	case B001:
		// a-hi c-lo
		analogWrite(pin_a_hi, speed);
		analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
		analogWrite(pin_b_hi, 0);
		analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
		analogWrite(pin_c_hi, 0);
		analogWrite(pin_c_lo, speed); // digitalWrite(pin_c_lo, HIGH)
		break;
	case B011:
		// b-hi c-lo
		analogWrite(pin_a_hi, 0);
		analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
		analogWrite(pin_b_hi, speed);
		analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
		analogWrite(pin_c_hi, 0);
		analogWrite(pin_c_lo, speed); // digitalWrite(pin_c_lo, HIGH)
		break;
	case B010:
		// b-hi a-lo
		analogWrite(pin_a_hi, 0);
		analogWrite(pin_a_lo, speed); // digitalWrite(pin_a_lo, HIGH)
		analogWrite(pin_b_hi, speed);
		analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
		analogWrite(pin_c_hi, 0);
		analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
		break;
	case B110:
		// c-hi a-lo
		analogWrite(pin_a_hi, 0);
		analogWrite(pin_a_lo, speed); // digitalWrite(pin_a_lo, HIGH)
		analogWrite(pin_b_hi, 0);
		analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
		analogWrite(pin_c_hi, speed);
		analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
		break;
	case B100:
		// c-hi b-lo
		analogWrite(pin_a_hi, 0);
		analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
		analogWrite(pin_b_hi, 0);
		analogWrite(pin_b_lo, speed); // digitalWrite(pin_b_lo, HIGH)
		analogWrite(pin_c_hi, speed);
		analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
		break;
	default: 
		// 000 111 error
		// fixme
		analogWrite(pin_a_hi, 0);
		analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
		analogWrite(pin_b_hi, 0);
		analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
		analogWrite(pin_c_hi, 0);
		analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
	}
	
}
			




			


void setup()
{
	pinMode(pin_a_hi, OUTPUT);
	pinMode(pin_a_lo, OUTPUT);
	pinMode(pin_b_hi, OUTPUT);
	pinMode(pin_b_lo, OUTPUT);
	pinMode(pin_c_hi, OUTPUT);
	pinMode(pin_c_lo, OUTPUT);
	
	
	digitalWrite(pin_a_hi, LOW);
	digitalWrite(pin_a_lo, LOW);
	digitalWrite(pin_b_hi, LOW);
	digitalWrite(pin_b_lo, LOW);
	digitalWrite(pin_c_hi, LOW);
	digitalWrite(pin_c_lo, LOW);
	
	
	
	pinMode(pin_a_cotrol_optocouple, OUTPUT);
	pinMode(pin_b_cotrol_optocouple, OUTPUT);
	pinMode(pin_c_cotrol_optocouple, OUTPUT);
	
	//analog_hall_level_detect();
}


void loop()
{
	turn_digital(1);

	delayMicroseconds(delay_between_step);
}
