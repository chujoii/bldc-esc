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



 Keywords: bldc esc hall bemf current motor sensor



 Usage:


 History:

 Version 0.1 was created at 2013.may.18



 Code:
*/

// phase a, b, c
int pin_a_hi = 3;
int pin_a_lo = 5;
int pin_b_hi = 6;
int pin_b_lo = 9;
int pin_c_hi = 10;
int pin_c_lo = 11;

// Hall a, b, c (or other type of sensor: for example optic)
/*
int analog_pin_a_hall = A0;
int analog_pin_b_hall = A1;
int analog_pin_c_hall = A2;
*/
int digital_pin_a_hall = 4;
int digital_pin_b_hall = 7;
int digital_pin_c_hall = 8;


// BEMF sensors
int analog_pin_a_bemf = A0;
int analog_pin_b_bemf = A1;
int analog_pin_c_bemf = A2;

// Current sensors
int analog_pin_a_current = A3;
int analog_pin_b_current = A4;
int analog_pin_c_current = A5;


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


const byte pwm_min = 0;
const byte pwm_max = 255;




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



void turn_analoghall(int diretcion) // not work
{
	// current state of hall sensor
	int state_a_hall, state_b_hall, state_c_hall;
	
	state_a_hall = analogRead(analog_pin_a_hall);
	state_b_hall = analogRead(analog_pin_b_hall);
	state_c_hall = analogRead(analog_pin_c_hall);
	
	int u_control_a_hi = map(state_a_hall, a_hall_min, a_hall_max, -pwm_max, pwm_max); // fixme: need use direction
	int u_control_a_lo = map(state_a_hall, a_hall_min, a_hall_max, -pwm_max, pwm_max); // fixme: need use direction

	int u_control_b_hi = map(state_a_hall, b_hall_min, b_hall_max, -pwm_max, pwm_max); // fixme: need use direction
	int u_control_b_lo = map(state_a_hall, b_hall_min, b_hall_max, -pwm_max, pwm_max); // fixme: need use direction

	int u_control_c_hi = map(state_a_hall, c_hall_min, c_hall_max, -pwm_max, pwm_max); // fixme: need use direction
	int u_control_c_lo = map(state_a_hall, c_hall_min, c_hall_max, -pwm_max, pwm_max); // fixme: need use direction

	//if (u_control_a<0)
	digitalWrite(pin_a_hi, abs(u_control_a_hi));
	digitalWrite(pin_a_lo, abs(u_control_a_lo));
	digitalWrite(pin_b_hi, abs(u_control_b_hi));
	digitalWrite(pin_b_lo, abs(u_control_b_lo));
	digitalWrite(pin_c_hi, abs(u_control_c_hi));
	digitalWrite(pin_c_lo, abs(u_control_c_lo));
}




void turn_digitalhall_case(int diretcion) // different number of compare "state_adc_hall"
{

	byte speed;
	
	speed = abc(direction);


	// current state of hall sensor
	int state_a_hall, state_b_hall, state_c_hall;
	byte state_abc_hall = 0;

	state_a_hall = digitalRead(digital_pin_a_hall);
	state_b_hall = digitalRead(digital_pin_b_hall);
	state_c_hall = digitalRead(digital_pin_c_hall);
	

	
	
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



	if (HIGH == state_a_hall) {state_abc_hall = state_abc_hall & B100};
	if (HIGH == state_b_hall) {state_abc_hall = state_abc_hall & B010};
	if (HIGH == state_c_hall) {state_abc_hall = state_abc_hall & B001};
	
	
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
			




void turn_digitalhall_if(int diretcion) // only 3 if ""
{

	byte speed;
	
	speed = abc(direction);


	// current state of hall sensor
	int state_a_hall, state_b_hall, state_c_hall;
	byte state_abc_hall = 0;

	state_a_hall = digitalRead(digital_pin_a_hall);
	state_b_hall = digitalRead(digital_pin_b_hall);
	state_c_hall = digitalRead(digital_pin_c_hall);
	

	
	
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


	
	if (HIGH == state_a_hall) {
		// 1xx ---------------------------------
		if (HIGH == state_b_hall) {
			// 11x -------------------------
			if (HIGH == state_c_hall) {
				// 111
				// error
				// fixme
				analogWrite(pin_a_hi, 0);
				analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
				analogWrite(pin_b_hi, 0);
				analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
				analogWrite(pin_c_hi, 0);
				analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
			} else {
				// 110
				// c-hi a-lo
				analogWrite(pin_a_hi, 0);
				analogWrite(pin_a_lo, speed); // digitalWrite(pin_a_lo, HIGH)
				analogWrite(pin_b_hi, 0);
				analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
				analogWrite(pin_c_hi, speed);
				analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
			}
		} else {
			// 10x -------------------------
			if (HIGH == state_c_hall) {
				// 101
				// a-hi b-lo
				analogWrite(pin_a_hi, speed);
				analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
				analogWrite(pin_b_hi, 0);
				analogWrite(pin_b_lo, speed); // digitalWrite(pin_b_lo, HIGH)
				analogWrite(pin_c_hi, 0);
				analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
			} else {
				// 100
				// c-hi b-lo
				analogWrite(pin_a_hi, 0);
				analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
				analogWrite(pin_b_hi, 0);
				analogWrite(pin_b_lo, speed); // digitalWrite(pin_b_lo, HIGH)
				analogWrite(pin_c_hi, speed);
				analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
			}
		}
	} else {
		// 0xx ---------------------------------
		if (HIGH == state_b_hall) {
			// 01x -------------------------
			if (HIGH == state_c_hall) {
				// 011
				// b-hi c-lo
				analogWrite(pin_a_hi, 0);
				analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
				analogWrite(pin_b_hi, speed);
				analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
				analogWrite(pin_c_hi, 0);
				analogWrite(pin_c_lo, speed); // digitalWrite(pin_c_lo, HIGH)
			} else {
				// 010
				// b-hi a-lo
				analogWrite(pin_a_hi, 0);
				analogWrite(pin_a_lo, speed); // digitalWrite(pin_a_lo, HIGH)
				analogWrite(pin_b_hi, speed);
				analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
				analogWrite(pin_c_hi, 0);
				analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
			}
		} else {
			// 00x -------------------------
			if (HIGH == state_c_hall) {
				// 001
				// a-hi c-lo
				analogWrite(pin_a_hi, speed);
				analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
				analogWrite(pin_b_hi, 0);
				analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
				analogWrite(pin_c_hi, 0);
				analogWrite(pin_c_lo, speed); // digitalWrite(pin_c_lo, HIGH)
			} else {
				// 000
				// error
				// fixme
				analogWrite(pin_a_hi, 0);
				analogWrite(pin_a_lo, 0); // digitalWrite(pin_a_lo, LOW)
				analogWrite(pin_b_hi, 0);
				analogWrite(pin_b_lo, 0); // digitalWrite(pin_b_lo, LOW)
				analogWrite(pin_c_hi, 0);
				analogWrite(pin_c_lo, 0); // digitalWrite(pin_c_lo, LOW)
			}
		}
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



	pinMode(pin_a_hi, OUTPUT);
	pinMode(pin_a_lo, OUTPUT);
	pinMode(pin_b_hi, OUTPUT);
	pinMode(pin_b_lo, OUTPUT);
	pinMode(pin_c_hi, OUTPUT);
	pinMode(pin_c_lo, OUTPUT);

	hall_level_detect();
}


void loop()
{
	turn(1);

	delay(1);
}


