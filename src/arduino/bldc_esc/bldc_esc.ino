/*
  bldc_esc.ino --- main program for bldc-esc

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

const boolean g_digital_abc_shift_cw  = true;         // !=0 true for cw
const boolean g_digital_abc_shift_ccw = false;        // ==0 false for ccw
float   g_analog_abc_shift_cw   = M_PI/6.0;     // 30[degree] for ccw
float   g_analog_abc_shift_ccw  = M_PI*7.0/6.0; // 210[degree] for cw


float g_old_analog_angle = 0.0;
byte g_old_digital_angle = 0;


long int g_turn_counter = 0;
unsigned long g_old_turn_timer_us = 0;
unsigned long g_turn_timer_us = 0;
unsigned long g_halfturn_timer_us = 0;


// fixme
float g_best_angle_abc_shift = 0;
long int g_best_turn_counter = 0;
long int g_old_turn_counter = 0;

// ----------------------------------- limit, ctrl --------------------------------

char g_main_ctrl_parameter = 'u';
int g_old_ctrl_value = 0;

const byte pwm_min = 0;
const byte pwm_max = 254; // 254 because driver high and low
                          // transistors - ir2101 does not contains
                          // generator, and use pwm from high logic
                          // input for charge bootstrap capacitor



const int analog_min = 0;
const int analog_max = 1023;



//int g_old_velocity_ctrl = pwm_min;
int g_velocity_ctrl = pwm_min;
float g_velocity_ctrl_proportional = 0.1;
float g_velocity_ctrl_integral = 0.0;
float g_velocity_ctrl_derivative = 0.0;

//int g_old_limit_velocity_ctrl = pwm_max;
int g_limit_speed_ctrl = pwm_max;



const int hard_limit_voltage = pwm_max;

//int g_old_voltage_ctrl = pwm_min;
int g_voltage_ctrl = pwm_min;   // motor not run in the start
float g_voltage_ctrl_proportional = 0.1;
float g_voltage_ctrl_integral = 0.0;
float g_voltage_ctrl_derivative = 0.0;

//int g_old_limit_voltage_ctrl = pwm_max;
int g_limit_voltage_ctrl = pwm_max;



const int hard_limit_current = analog_max/2;

//int g_old_current_ctrl = pwm_min;
int g_current_ctrl = pwm_min;
float g_current_ctrl_proportional = 0.1;
float g_current_ctrl_integral = 0.0;
float g_current_ctrl_derivative = 0.0;

//int g_old_limit_current_ctrl = pwm_max;
int g_limit_current_ctrl = pwm_max;



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

char g_algorithm = 'a'; // a - analog
                        // d - digital


const int delay_between_step = 1; // us

const unsigned long print_dt = 200;
unsigned long g_time_to_print = 0;

char g_cmd_line [50];
int g_cmd_line_max_length = 50;
int g_cmd_line_length = 0;

#include "/home/chujoii/project/bldc-esc/src/angle-calculation.c"


boolean sign (int x)
{
	if (x>=0) {return true;} else {return false;}
}


float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}






			


void setup()
{
	pinMode(pin_phase_a_hi, OUTPUT);
	pinMode(pin_phase_a_lo, OUTPUT);
	pinMode(pin_phase_b_hi, OUTPUT);
	pinMode(pin_phase_b_lo, OUTPUT);
	pinMode(pin_phase_c_hi, OUTPUT);
	pinMode(pin_phase_c_lo, OUTPUT);
	
	
	free_rotation();
	
	
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
	if (g_algorithm == 'd'){
		turn_digital(apply_pid(), digital_read_angle());
	} else {
		turn_analog(apply_pid(), analog_read_angle(), g_analog_abc_shift_cw, g_analog_abc_shift_ccw);
	}



	if (millis() > g_time_to_print){
		g_time_to_print = millis() + print_dt;
		

		sprintf (buffer, "current = %d\t", read_abc_current());
		Serial.print(buffer);

		//find_best_angle_shift();

		//g_analog_abc_shift_cw = fmap((float)analogRead(A4), 0.0, 1023.0, 0.0, 6.3);
		//Serial.print("angle_shift = "); Serial.print(g_analog_abc_shift_cw); Serial.print("\t"); // only for cw (velocity > 0)

		read_ctrl();

		Serial.print("rpm = "); Serial.print(get_rpm());
		
		
		//Serial.print("old_turn_time = "); Serial.print(g_old_turn_timer_us);
		Serial.print("\tturn_time = "); Serial.print((g_turn_timer_us - g_old_turn_timer_us)/1000);
		Serial.print("\thalf_turn_time = "); Serial.print((g_halfturn_timer_us - g_old_turn_timer_us)/1000);
		Serial.print("\tmax_turn_time = "); Serial.print((max(g_turn_timer_us - g_old_turn_timer_us, g_halfturn_timer_us - g_old_turn_timer_us))/1000);


		Serial.print("\told_angle = "); Serial.print(g_old_analog_angle);
		Serial.print("\tangle = "); Serial.print(analog_read_angle());


		Serial.print("\tturn_counter = "); Serial.print(g_turn_counter);



		
	        Serial.println();
	}

	
	//Serial.println();
	//delay(200);
	delayMicroseconds(delay_between_step);
}
