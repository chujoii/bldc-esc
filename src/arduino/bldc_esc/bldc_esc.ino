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


//#define DEBUGA


#ifdef DEBUGA
 #define DEBUGA_PRINT(x)  Serial.print (x)
#else
 #define DEBUGA_PRINT(x)
#endif

char buffer [50];





// global interrupt pin
const int pin_global_interrupt = 2;

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

const float   g_analog_abc_shift_cw   = M_PI/6.0;     // 30[degree] for ccw
const float   g_analog_abc_shift_ccw  = M_PI*7.0/6.0; // 210[degree] for cw
const boolean g_digital_abc_shift_cw  = true;         // !=0 true for cw
const boolean g_digital_abc_shift_ccw = false;        // ==0 false for ccw
float g_analog_abc_fine_tune_angle_shift = 0.0;                       // to fine-tune the angle of shift

float g_old_analog_angle = 0.0;
byte g_old_digital_angle = 0;


long int g_turn_counter = 0;

// fixme
float g_best_angle_abc_shift = 0;
long int g_best_turn_counter = 0;
long int g_old_turn_counter = 0;




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
const int analog_pin_x_hall = A0;
const int analog_pin_y_hall = A1;
const int analog_pin_z_hall = A2;

int analog_pin_a_hall = analog_pin_x_hall;
int analog_pin_b_hall = analog_pin_y_hall;
int analog_pin_c_hall = analog_pin_z_hall;


const int analog_min = 0;
const int analog_max = 1023;

// hall max min level
int g_hall_max  = analog_min;
int g_hall_min  = analog_max;
int g_hall_zero = (analog_max + analog_min)/2;

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

const unsigned long print_dt = 200;
unsigned long g_time_to_print = 0;


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

boolean sign (int x)
{
	if (x>=0) {return true;} else {return false;}
}

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



byte digital_read_all_hall_sensors()
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


int read_abc_current()
{
	return analogRead(analog_pin_abc_current) - g_zero_abc_current;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




int search_pinout(int speed, unsigned int waiting_time, int change_limit, char test_phase)
{
	int phase_num_1, phase_num_2;
	
	int old_x = analog_read_hall_sensor(analog_pin_x_hall);
	int old_y = analog_read_hall_sensor(analog_pin_y_hall);
	int old_z = analog_read_hall_sensor(analog_pin_z_hall);

	
	int x = old_x + change_limit;
	int y = old_y + change_limit;
	int z = old_z + change_limit;

	switch (test_phase) {
		/*
		  B101 = 5 Ab-
		  B001 = 1 A-c
		  B011 = 3 -Bc
		  B010 = 2 aB-
		  B110 = 6 a-C
		  B100 = 4 -bC
		*/
	case 'a':
		phase_num_1 = B101;
		phase_num_2 = B001;
		break;
	case 'b':
		phase_num_1 = B011;
		phase_num_2 = B010;
		break;
	case 'c':
	default:
		phase_num_1 = B110;
		phase_num_2 = B100;
	}

	
	boolean one_or_two = true;

	unsigned int delay_time = waiting_time;
	unsigned int timer = millis() + waiting_time;
	
	// wait for stop changes
	while((abs(x - old_x) + abs(y - old_y) + abs(z - old_z) > change_limit) && (delay_time > 0)){
		if (one_or_two) {
			turn_digital(speed, phase_num_1);
			one_or_two = !one_or_two;
		} else {
			turn_digital(speed, phase_num_2);
			one_or_two = !one_or_two;
		}
		old_x = x;
		old_y = y;
		old_z = z;
		delay(delay_time);
		x = analog_read_hall_sensor(analog_pin_x_hall);
		y = analog_read_hall_sensor(analog_pin_y_hall);
		z = analog_read_hall_sensor(analog_pin_z_hall);
		//sprintf (buffer, "x=%d\ty=%d\tz=%d\tdiff=%d", x, y, z, abs(x - old_x) + abs(y - old_y) + abs(z - old_z));
		//Serial.println(buffer);
		delay(delay_time);

		if (millis() > timer){
			delay_time--;
			timer = millis() + waiting_time;
		}
	}

	Serial.println(delay_time);
	
	if (x > y && x > z) {return analog_pin_x_hall;}
	if (y > x && y > z) {return analog_pin_y_hall;}
	if (z > x && z > y) {return analog_pin_z_hall;}
	return analog_pin_z_hall; // strange
}


void stop_motor()
{
	digitalWrite(pin_phase_a_hi, LOW);
	digitalWrite(pin_phase_a_lo, LOW);
	digitalWrite(pin_phase_b_hi, LOW);
	digitalWrite(pin_phase_b_lo, LOW);
	digitalWrite(pin_phase_c_hi, LOW);
	digitalWrite(pin_phase_c_lo, LOW);
}

void turn_analog(int velocity)
{
	int speed; // speed = scalar absolute value (magnitude) of velocity
	speed = abs(velocity);

	float angle_shift;
	
	if (velocity>0) {
		angle_shift = g_analog_abc_shift_cw  + g_analog_abc_fine_tune_angle_shift;
	} else {
		angle_shift = g_analog_abc_shift_ccw + g_analog_abc_fine_tune_angle_shift;
	}
	
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
	
	float phase_a_val = sin(angle + angle_a_shift + angle_shift);
	float phase_b_val = sin(angle + angle_b_shift + angle_shift);
	float phase_c_val = sin(angle + angle_c_shift + angle_shift);

	if (phase_a_val>0.0) {
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite (pin_phase_a_hi, (int)fmap(abs(phase_a_val), 0.0, 1.0, pwm_min, speed));
		if (phase_a_val<0.5){DEBUGA_PRINT("(-");}else{DEBUGA_PRINT("(A");}
		//DEBUGA_PRINT((int)fmap(abs(phase_a_val), 0.0, 1.0, pwm_min, speed));DEBUGA_PRINT("\t");
	} else {
		digitalWrite(pin_phase_a_hi, LOW);
		analogWrite (pin_phase_a_lo, (int)fmap(abs(phase_a_val), 0.0, 1.0, pwm_min, speed));
		if (phase_a_val>0.5){DEBUGA_PRINT("(-");}else{DEBUGA_PRINT("(a");}
		//DEBUGA_PRINT((int)fmap(abs(phase_a_val), 0.0, 1.0, pwm_min, speed));DEBUGA_PRINT("\t");
	}

	if (phase_b_val>0.0) {
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite (pin_phase_b_hi, (int)fmap(abs(phase_b_val), 0.0, 1.0, pwm_min, speed));
		if (phase_b_val<0.5){DEBUGA_PRINT("-");}else{DEBUGA_PRINT("B");}
		//DEBUGA_PRINT((int)fmap(abs(phase_b_val), 0.0, 1.0, pwm_min, speed));DEBUGA_PRINT("\t");
	} else {
		digitalWrite(pin_phase_b_hi, LOW);
		analogWrite (pin_phase_b_lo, (int)fmap(abs(phase_b_val), 0.0, 1.0, pwm_min, speed));
		if (phase_b_val>0.5){DEBUGA_PRINT("-");}else{DEBUGA_PRINT("b");}
		//DEBUGA_PRINT((int)fmap(abs(phase_b_val), 0.0, 1.0, pwm_min, speed));DEBUGA_PRINT("\t");
	}
	
	if (phase_c_val>0.0) {
		digitalWrite(pin_phase_c_lo, LOW);
		analogWrite (pin_phase_c_hi, (int)fmap(abs(phase_c_val), 0.0, 1.0, pwm_min, speed));
		if (phase_c_val<0.5){DEBUGA_PRINT("-)\t");}else{DEBUGA_PRINT("C)\t");}
		//DEBUGA_PRINT((int)fmap(abs(phase_c_val), 0.0, 1.0, pwm_min, speed));DEBUGA_PRINT("\t");
	} else {
		digitalWrite(pin_phase_c_hi, LOW);
		analogWrite (pin_phase_c_lo, (int)fmap(abs(phase_c_val), 0.0, 1.0, pwm_min, speed));
		if (phase_c_val>0.5){DEBUGA_PRINT("-)\t");}else{DEBUGA_PRINT("c)\t");}
		//DEBUGA_PRINT((int)fmap(abs(phase_c_val), 0.0, 1.0, pwm_min, speed));DEBUGA_PRINT("\t");
	}

	// fixme: need minimum 3 measurement by each turn:
	// what happend if number of measurement less than 3?
	// rigth answer motor stop because control become incorrect
	//
	//  g_old_analog_angle < 120[degree]     &&  angle > 240[degree]
	if (g_old_analog_angle < angle_b_shift   &&  angle > angle_c_shift){ g_turn_counter++;}
	//
	//  g_old_analog_angle > 240[degree]     &&  angle < 120[degree]
	if (g_old_analog_angle > angle_c_shift   &&  angle < angle_b_shift){ g_turn_counter--;}
	
	g_old_analog_angle = angle;

}




void turn_digital(int velocity, byte digital_angle)
{
	/*
	  digital_angle:
	  B101 = 5 Ab-
	  B001 = 1 A-c
	  B011 = 3 -Bc
	  B010 = 2 aB-
	  B110 = 6 a-C
	  B100 = 4 -bC
	*/
	
	byte speed; // speed = scalar absolute value (magnitude) of velocity
	speed = abs(velocity);

	
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

	  101         a-b       a-hi b-lo
	  001         a-c       a-hi c-lo
	  011         b-c       b-hi c-lo
	  010         b-a       b-hi a-lo
	  110         c-a       c-hi a-lo
	  100         c-b       c-hi b-lo

	  111         error
	  000         error

	 */


	if (sign(velocity) == g_digital_abc_shift_cw){
		// fixme
		switch (digital_angle) {
		case B101:
			digital_angle = B001;
			break;
		case B001:
			digital_angle = B011;
			break;
		case B011:
			digital_angle = B010;
			break;
		case B010:
			digital_angle = B110;
			break;
		case B110:
			digital_angle = B100;
			break;
		case B100:
			digital_angle = B101;
			break;
		}
	}









	
	
	switch (digital_angle) {
	case B101:
		//a-hi b-lo
		DEBUGA_PRINT("[Ab-]\t");
		analogWrite(pin_phase_a_hi, speed);
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite(pin_phase_b_hi, 0);
		digitalWrite(pin_phase_b_lo, HIGH);
		analogWrite(pin_phase_c_hi, 0);
		digitalWrite(pin_phase_c_lo, LOW);
		break;
	case B001:
		// a-hi c-lo
		DEBUGA_PRINT("[A-c]\t");
		analogWrite(pin_phase_a_hi, speed);
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite(pin_phase_b_hi, 0);
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite(pin_phase_c_hi, 0);
		digitalWrite(pin_phase_c_lo, HIGH);
		break;
	case B011:
		// b-hi c-lo
		DEBUGA_PRINT("[-Bc]\t");
		analogWrite(pin_phase_a_hi, 0);
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite(pin_phase_b_hi, speed);
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite(pin_phase_c_hi, 0);
		digitalWrite(pin_phase_c_lo, HIGH);
		break;
	case B010:
		// b-hi a-lo
		DEBUGA_PRINT("[aB-]\t");
		analogWrite(pin_phase_a_hi, 0);
		digitalWrite(pin_phase_a_lo, HIGH);
		analogWrite(pin_phase_b_hi, speed);
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite(pin_phase_c_hi, 0);
		digitalWrite(pin_phase_c_lo, LOW);
		break;
	case B110:
		// c-hi a-lo
		DEBUGA_PRINT("[a-C]\t");
		analogWrite(pin_phase_a_hi, 0);
		digitalWrite(pin_phase_a_lo, HIGH);
		analogWrite(pin_phase_b_hi, 0);
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite(pin_phase_c_hi, speed);
		digitalWrite(pin_phase_c_lo, LOW);
		break;
	case B100:
		// c-hi b-lo
		DEBUGA_PRINT("[-bC]\t");
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
		DEBUGA_PRINT("[---]\t");
		analogWrite(pin_phase_a_hi, 0);
		digitalWrite(pin_phase_a_lo, LOW);
		analogWrite(pin_phase_b_hi, 0);
		digitalWrite(pin_phase_b_lo, LOW);
		analogWrite(pin_phase_c_hi, 0);
		digitalWrite(pin_phase_c_lo, LOW);
	}

	if (g_old_digital_angle == B101 && digital_angle == B100){ g_turn_counter++;}
	if (g_old_digital_angle == B100 && digital_angle == B101){ g_turn_counter--;}
	g_old_digital_angle = digital_angle;
}


void find_best_angle_shift(){
		if (abs(g_turn_counter - g_old_turn_counter) > g_best_turn_counter) {
			g_best_turn_counter = abs(g_turn_counter - g_old_turn_counter);
			g_best_angle_abc_shift = g_analog_abc_fine_tune_angle_shift;
		}
		

		g_analog_abc_fine_tune_angle_shift = g_analog_abc_fine_tune_angle_shift + 0.01;

		
		//DEBUGA_PRINT("angle_shift = "); DEBUGA_PRINT(g_analog_abc_fine_tune_angle_shift); DEBUGA_PRINT("\t"); // strange sprintf not work

		//DEBUGA_PRINT("best_angle_shift = "); DEBUGA_PRINT(g_best_angle_abc_shift); DEBUGA_PRINT("\t"); // strange sprintf not work

		//sprintf (buffer, "turn_diff = %d\t", abs(g_turn_counter - g_old_turn_counter));
		//DEBUGA_PRINT(buffer);
		
		//sprintf (buffer, "best_turn = %d\t", g_best_turn_counter);
		//DEBUGA_PRINT(buffer);


		//sprintf (buffer, "turn = %d\t", g_turn_counter);
		//DEBUGA_PRINT(buffer);



		// simple 2 column
		DEBUGA_PRINT(g_analog_abc_fine_tune_angle_shift); DEBUGA_PRINT("\t"); // strange sprintf not work
		sprintf (buffer, "%ld", g_turn_counter - g_old_turn_counter);
		DEBUGA_PRINT(buffer);
		
		
		g_old_turn_counter = g_turn_counter;
}


			


void setup()
{
	pinMode(pin_phase_a_hi, OUTPUT);
	pinMode(pin_phase_a_lo, OUTPUT);
	pinMode(pin_phase_b_hi, OUTPUT);
	pinMode(pin_phase_b_lo, OUTPUT);
	pinMode(pin_phase_c_hi, OUTPUT);
	pinMode(pin_phase_c_lo, OUTPUT);
	
	
	stop_motor();
	
	
	/*
	pinMode(pin_a_cotrol_optocouple, OUTPUT);
	pinMode(pin_b_cotrol_optocouple, OUTPUT);
	pinMode(pin_c_cotrol_optocouple, OUTPUT);
	*/

	delay(1000);

	Serial.begin(115200);
	
	//analog_hall_level_detect();

	g_zero_abc_current = read_abc_current();

	
	do {
		analog_pin_a_hall = search_pinout(10, 30, 1, 'a');
		sprintf (buffer, "for phase A sensor pin = %d", analog_pin_a_hall);
		Serial.println(buffer);
		
		analog_pin_b_hall = search_pinout(10, 30, 1, 'b');
		sprintf (buffer, "for phase B sensor pin = %d", analog_pin_b_hall);
		Serial.println(buffer);
		
		analog_pin_c_hall = search_pinout(10, 30, 1, 'c');
		sprintf (buffer, "for phase C sensor pin = %d", analog_pin_c_hall);
		Serial.println(buffer);
	} while ((analog_pin_a_hall == analog_pin_b_hall) || (analog_pin_b_hall == analog_pin_c_hall) || (analog_pin_c_hall == analog_pin_a_hall));



	
	
	stop_motor();
}


void loop()
{
	turn_analog(20);
	//turn_digital(10, digital_read_all_hall_sensors());


	if (millis() > g_time_to_print){
		g_time_to_print = millis() + print_dt;
		

		//sprintf (buffer, "current = %d\t", read_abc_current());
		//DEBUGA_PRINT(buffer);

		//find_best_angle_shift();

		//g_analog_abc_fine_tune_angle_shift = fmap((float)analogRead(A4), 0.0, 1023.0, 0.0, 6.3);
		//Serial.print("angle_shift = "); Serial.print(g_analog_abc_fine_tune_angle_shift); Serial.print("\t"); // strange sprintf not work

		
		//Serial.println();
	}

	
	//Serial.println();
	//delay(200);
	delayMicroseconds(delay_between_step);
}
