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


boolean debugging = true;
char buffer [50];





// global interrupt pin
int pin_global_interrupt = 2;

const float angle_a_shift = 0.0;            //  0.0[degree]
const float angle_b_shift = M_PI * 2.0/3.0; //120.0[degree]
const float angle_c_shift = M_PI * 4.0/3.0; //240.0[degree]

const float point_of_symmetry_sin_x_plus  = M_PI / 2.0;     // 90.0[degree]
const float point_of_symmetry_sin_x_minus = M_PI * 3.0/2.0; //270.0[degree]
const float point_of_zero_cross_sin_x     = M_PI;           //180.0[degree]
const float point_of_cycle_min            = 0.0;            //  0.0[degree]
const float point_of_cycle_max            = M_PI * 2.0;     //360.0[degree]

const float min_sin_val = -1.0;
const float max_sin_val =  1.0;

const float epsilon = 0.01;

float g_angle_abc_shift = -M_PI/4.0; // -360[degree]


// -------------------------------------- pins --------------------------------------
/* http://arduino.cc/en/Main/ArduinoBoardDuemilanove
   On boards other than the Mega, use of the Servo library disables analogWrite() (PWM) functionality on pins 9 and 10.
   
   pin  - capability     - function   (if pin without [] == NotConnected)
   0    - rx             - 
   1    - tx             - 
   2    -      ext_int 0 - 
  [3]   - pwm, ext_int 1 - c_lo
   4    -                - 
  [5]   - pwm            - c_hi
  [6]   - pwm            - b_lo
   7    -                - 
   8    -                - 
  [9]   - {pwm}          - b_hi
  [10]  - {pwm}          - a_lo
  [11]  - pwm            - a_hi
   12   -                - 
   13   - led            - 

  [A0]  -                - a hall
  [A1]  -                - b hall
  [A2]  -                - c hall
  [A3]  -                - abc current
   A4   - sda            - 
   A5   - scl            - 
*/

const int pin_phase_a_hi = 11; // pwm
const int pin_phase_a_lo = 10; // pwm
const int pin_phase_b_hi = 9;  // pwm
const int pin_phase_b_lo = 6;  // pwm
const int pin_phase_c_hi = 5;  // pwm
const int pin_phase_c_lo = 3;  // pwm

// ///////////////////////////////////////////// sensors ////////

// Hall a, b, c (or other type of sensor: for example optic)
const int analog_pin_a_hall = A0;
const int analog_pin_b_hall = A1;
const int analog_pin_c_hall = A2;

//const int digital_pin_a_hall = 4;
//const int digital_pin_b_hall = 7;
//const int digital_pin_c_hall = 8;


const int analog_min = 0;
const int analog_max = 1023;
// hall max min level
int hall_max  = analog_min;
int hall_min  = analog_max;
int hall_zero = (analog_max + analog_min)/2;

//const int analog_pin_a_optocouple = A0;
//const int analog_pin_b_optocouple = A1;
//const int analog_pin_c_optocouple = A2;

//const int pin_a_cotrol_optocouple = 4;
//const int pin_b_cotrol_optocouple = 7;
//const int pin_c_cotrol_optocouple = 8;

//const byte optocouple_delay = 1; // us   fixme: need experiments
//int optocouple_threshold_level = 128; // fixme: need experiments


// BEMF sensors
//const int analog_pin_a_bemf = A0;
//const int analog_pin_b_bemf = A1;
//const int analog_pin_c_bemf = A2;

// Current sensors
//const int analog_pin_a_current = A3;
//const int analog_pin_b_current = A4;
//const int analog_pin_c_current = A5;
const int analog_pin_abc_current = A3;
int g_zero_abc_current = 0;



const byte pwm_min = 0;
const byte pwm_max = 254; // 254 because driver high and low
                          // transistors - ir2101 does not contains
                          // generator, and use pwm from high logic
                          // input for charge bootstrap capacitor


const int delay_between_step = 1; // us

const unsigned long print_dt = 100;
unsigned long time_to_print = 0;


#include "/home/chujoii/project/bldc-esc/src/angle-calculation.c"

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
	int hall_val = analogRead(pin_hall);
	if (hall_max < hall_val) {hall_max = hall_val;}
	if (hall_min > hall_val) {hall_min = hall_val;}
	hall_zero = (hall_max + hall_min)/2;

	//sprintf (buffer, "h = %d (%d)\t", hall_val, hall_val > a_hall_zero);
	//Serial.print(buffer);

	return hall_val > hall_zero;
}

int analog_read_hall_sensor(int pin_hall)
{
	int hall_val = analogRead(pin_hall);
	if (hall_max < hall_val) {hall_max = hall_val;}
	if (hall_min > hall_val) {hall_min = hall_val;}
	hall_zero = (hall_max + hall_min)/2;

	return hall_val;
}


int read_abc_current()
{
	return analogRead(analog_pin_abc_current) - g_zero_abc_current;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void turn_analog(int direction)
{
	int speed;
	speed = abs(direction);
	
	
        // current state of hall sensor
	int state_a_hall, state_b_hall, state_c_hall;


	state_a_hall = analog_read_hall_sensor(analog_pin_a_hall);
	state_b_hall = analog_read_hall_sensor(analog_pin_b_hall);
	state_c_hall = analog_read_hall_sensor(analog_pin_c_hall);

	float a1 = state_a_hall;
	float b1 = state_b_hall;
	float c1 = state_c_hall;
	float hmin = hall_min;
	float hmax = hall_max;

	float normalize_a_hall = fmap(a1, hmin, hmax, -1.0, 1.0);
	//float normalize_a_hall = fmap((float)state_a_hall, (float)hall_min, (float)hall_max, -1.0, 1.0);
	float normalize_b_hall = fmap((float)state_b_hall, (float)hall_min, (float)hall_max, -1.0, 1.0);
	float normalize_c_hall = fmap((float)state_c_hall, (float)hall_min, (float)hall_max, -1.0, 1.0);

	float angle = calculation_angle_from_three_phases(normalize_a_hall, normalize_b_hall, normalize_c_hall);

	//Serial.print(angle);
	//Serial.print("\t");
	
	float phase_a_val = sin(angle - angle_a_shift + g_angle_abc_shift);
	float phase_b_val = sin(angle - angle_b_shift + g_angle_abc_shift);
	float phase_c_val = sin(angle - angle_c_shift + g_angle_abc_shift);

	if (phase_a_val>0.0) {
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite (pin_phase_a_hi, (int)fmap(abs(phase_a_val), 0.0, 1.0, pwm_min, speed));
		//Serial.print("A");
		//Serial.print((int)fmap(abs(phase_a_val), 0.0, 1.0, pwm_min, speed));Serial.print("\t");
	} else {
		digitalWrite(pin_phase_a_hi, LOW);
		analogWrite (pin_phase_a_lo, (int)fmap(abs(phase_a_val), 0.0, 1.0, pwm_min, speed));
		//Serial.print("a");
		//Serial.print((int)fmap(abs(phase_a_val), 0.0, 1.0, pwm_min, speed));Serial.print("\t");
	}

	if (phase_b_val>0.0) {
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite (pin_phase_b_hi, (int)fmap(abs(phase_b_val), 0.0, 1.0, pwm_min, speed));
		//Serial.print("B");
		//Serial.print((int)fmap(abs(phase_b_val), 0.0, 1.0, pwm_min, speed));Serial.print("\t");
	} else {
		digitalWrite(pin_phase_b_hi, LOW);
		analogWrite (pin_phase_b_lo, (int)fmap(abs(phase_b_val), 0.0, 1.0, pwm_min, speed));
		//Serial.print("b");
		//Serial.print((int)fmap(abs(phase_b_val), 0.0, 1.0, pwm_min, speed));Serial.print("\t");
	}
	
	if (phase_c_val>0.0) {
		digitalWrite(pin_phase_c_lo, LOW);
		analogWrite (pin_phase_c_hi, (int)fmap(abs(phase_c_val), 0.0, 1.0, pwm_min, speed));
		//Serial.print("C");
		//Serial.print((int)fmap(abs(phase_c_val), 0.0, 1.0, pwm_min, speed));Serial.print("\t");
	} else {
		digitalWrite(pin_phase_c_hi, LOW);
		analogWrite (pin_phase_c_lo, (int)fmap(abs(phase_c_val), 0.0, 1.0, pwm_min, speed));
		//Serial.print("c");
		//Serial.print((int)fmap(abs(phase_c_val), 0.0, 1.0, pwm_min, speed));Serial.print("\t");
	}
}




void turn_digital(int direction)
{

	byte speed;
	
	speed = abs(direction);


	// current state of hall sensor
	boolean state_a_hall, state_b_hall, state_c_hall;
	byte state_abc_hall = 0;

	state_a_hall = digital_read_hall_sensor(analog_pin_a_hall);
	state_b_hall = digital_read_hall_sensor(analog_pin_b_hall);
	state_c_hall = digital_read_hall_sensor(analog_pin_c_hall);
	
	state_abc_hall = (state_c_hall<<2) | (state_b_hall<<1) | state_a_hall;
	
	/*
	  Switches commutation for rotor rotation



	  ahall   ---___

	  bhall   __---_

	  chall   -___--

	          __   
	  aphase    -__-

                    __
	  bphase  _-  -_

	              __
	  cphase  -__-


	  ahi           bhi           chi
	  alo           blo           clo
	  

	  hall        phase     switches
          cab
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
		//a-hi b-lo
		analogWrite(pin_phase_a_hi, speed);
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite(pin_phase_b_hi, 0);
		digitalWrite(pin_phase_b_lo, HIGH);
		analogWrite(pin_phase_c_hi, 0);
		digitalWrite(pin_phase_c_lo, LOW);
		break;
	case B001:
		// a-hi c-lo
		analogWrite(pin_phase_a_hi, speed);
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite(pin_phase_b_hi, 0);
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite(pin_phase_c_hi, 0);
		digitalWrite(pin_phase_c_lo, HIGH);
		break;
	case B011:
		// b-hi c-lo
		analogWrite(pin_phase_a_hi, 0);
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite(pin_phase_b_hi, speed);
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite(pin_phase_c_hi, 0);
		digitalWrite(pin_phase_c_lo, HIGH);
		break;
	case B010:
		// b-hi a-lo
		analogWrite(pin_phase_a_hi, 0);
		digitalWrite(pin_phase_a_lo, HIGH);
		analogWrite(pin_phase_b_hi, speed);
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite(pin_phase_c_hi, 0);
		digitalWrite(pin_phase_c_lo, LOW);
		break;
	case B110:
		// c-hi a-lo
		analogWrite(pin_phase_a_hi, 0);
		digitalWrite(pin_phase_a_lo, HIGH);
		analogWrite(pin_phase_b_hi, 0);
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite(pin_phase_c_hi, speed);
		digitalWrite(pin_phase_c_lo, LOW);
		break;
	case B100:
		// c-hi b-lo
		analogWrite(pin_phase_a_hi, 0);
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite(pin_phase_b_hi, 0);
		digitalWrite(pin_phase_b_lo, HIGH);
		analogWrite(pin_phase_c_hi, speed);
		digitalWrite(pin_phase_c_lo, LOW);
		break;
	default:
		// 000 111 error
		// fixme
		analogWrite(pin_phase_a_hi, 0);
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite(pin_phase_b_hi, 0);
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite(pin_phase_c_hi, 0);
		digitalWrite(pin_phase_c_lo, LOW);
	}
}
			




			


void setup()
{
	pinMode(pin_phase_a_hi, OUTPUT);
	pinMode(pin_phase_a_lo, OUTPUT);
	pinMode(pin_phase_b_hi, OUTPUT);
	pinMode(pin_phase_b_lo, OUTPUT);
	pinMode(pin_phase_c_hi, OUTPUT);
	pinMode(pin_phase_c_lo, OUTPUT);
	
	
	digitalWrite(pin_phase_a_hi, LOW);
	digitalWrite(pin_phase_a_lo, LOW);
	digitalWrite(pin_phase_b_hi, LOW);
	digitalWrite(pin_phase_b_lo, LOW);
	digitalWrite(pin_phase_c_hi, LOW);
	digitalWrite(pin_phase_c_lo, LOW);
	
	
	/*
	pinMode(pin_a_cotrol_optocouple, OUTPUT);
	pinMode(pin_b_cotrol_optocouple, OUTPUT);
	pinMode(pin_c_cotrol_optocouple, OUTPUT);
	*/

	delay(1000);

	Serial.begin(115200);
	
	//analog_hall_level_detect();
	g_zero_abc_current = read_abc_current();

}


void loop()
{
	//turn_digital(10);
	turn_analog(20);

	//sprintf (buffer, "current = %d\t", read_abc_current());
	//Serial.print(buffer);

	// current state of hall sensor
	int state_a_hall, state_b_hall, state_c_hall;




	
	
	if (millis() > time_to_print){
		time_to_print = millis() + print_dt;
		g_angle_abc_shift = g_angle_abc_shift + 0.001;
		//Serial.print(g_angle_abc_shift);
		//Serial.println();
	}
	
	delayMicroseconds(delay_between_step);
}
