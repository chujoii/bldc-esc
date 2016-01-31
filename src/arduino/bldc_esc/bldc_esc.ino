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






const float SHIFT_ANGLE_A = 0.0;            //  0.0[degree]
const float SHIFT_ANGLE_B = M_PI * 2.0/3.0; //120.0[degree]
const float SHIFT_ANGLE_C = M_PI * 4.0/3.0; //240.0[degree]

const float POINT_OF_SYMMETRY_SIN_X_PLUS  = M_PI / 2.0;     // 90.0[degree]
const float POINT_OF_SYMMETRY_SIN_X_MINUS = M_PI * 3.0/2.0; //270.0[degree]
const float POINT_OF_ZERO_CROSS_SIN_X     = M_PI;           //180.0[degree]
const float POINT_OF_CYCLE_SIN_X_MIN            = 0.0;            //  0.0[degree]
const float POINT_OF_CYCLE_SIN_X_MAX            = M_PI * 2.0;     //360.0[degree]

const float SIN_VAL_MIN = -1.0;
const float SIN_VAL_MAX =  1.0;

const float EPSILON_ANGLE = 0.01;

const boolean G_DIGITAL_ABC_SHIFT_CW  = true;         // !=0 true for cw
const boolean G_DIGITAL_ABC_SHIFT_CCW = false;        // ==0 false for ccw
float   g_analog_abc_shift_cw   = M_PI/6.0;     // 30[degree] for ccw
float   g_analog_abc_shift_ccw  = M_PI*7.0/6.0; // 210[degree] for cw





// ----------------------------------- limit, ctrl --------------------------------

char g_main_ctrl_parameter = 'u';
int g_old_ctrl_value = 0;

const byte DAC_MIN = 0;
const byte DAC_MAX = 255;
const byte DAC_DRIVER_MAX = 254; // maximum of pwm = 254 (not 255)
		 		 // because driver high and low
		 		 // transistors - ir2101 - does not
		 		 // contains generator, and use pwm
		 		 // from high logic input for charge
		 		 // bootstrap capacitor



const int ADC_MIN = 0;
const int ADC_MAX = 1023;

const int STATISTIC_MEAN = 1000;
const int VALUE_ERR = 20;


//int g_old_velocity_ctrl = DAC_MIN;
int g_velocity_ctrl = DAC_MIN;
float g_velocity_ctrl_proportional = 0.1;
float g_velocity_ctrl_integral = 0.0;
float g_velocity_ctrl_derivative = 0.0;

//int g_old_limit_velocity_ctrl = DAC_MAX;
int g_limit_speed_ctrl = DAC_MAX;





//int g_old_voltage_ctrl = DAC_MIN;
int g_voltage_ctrl = DAC_MIN;   // motor not run in the start
float g_voltage_ctrl_proportional = 0.1;
float g_voltage_ctrl_integral = 0.0;
float g_voltage_ctrl_integral_old_val = 0.0;
float g_voltage_ctrl_derivative = 0.0;
float g_voltage_ctrl_derivative_old_val = 0.0;
unsigned long g_voltage_ctrl_derivative_old_t = 0.0;

//int g_old_limit_voltage_ctrl = DAC_MAX;
int g_limit_voltage_ctrl = DAC_MAX;



const int HARD_LIMIT_CURRENT = ADC_MAX/2;

//int g_old_current_ctrl = DAC_MIN;
int g_current_ctrl = DAC_MIN;
float g_current_ctrl_proportional = 0.1;
float g_current_ctrl_integral = 0.0;
float g_current_ctrl_integral_old_val = 0.0;
float g_current_ctrl_derivative = 0.0;
float g_current_ctrl_derivative_old_val = 0.0;
unsigned long g_current_ctrl_derivative_old_t = 0.0;

//int g_old_limit_current_ctrl = DAC_MAX;
int g_limit_current_ctrl = DAC_MAX;




	int g_real_direction = 1; // +1 = cw;         -1 = ccw
	
        // fixme
	float g_best_angle_abc_shift = 0;
	long int g_best_turn_counter = 0;
	long int g_old_turn_counter = 0;
	unsigned long g_turn_timer_us = 0;


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

const int PIN_PHASE_A_HI = 11; // pwm
const int PIN_PHASE_A_LO = 10; // pwm
const int PIN_PHASE_B_HI = 9;  // pwm
const int PIN_PHASE_B_LO = 6;  // pwm
const int PIN_PHASE_C_HI = 5;  // pwm
const int PIN_PHASE_C_LO = 3;  // pwm

// ///////////////////////////////////////////// sensors ////////

// Hall a, b, c (or other type of sensor: for example optic)
const int PIN_ANALOG_A_HALL = A0;
const int PIN_ANALOG_B_HALL = A1;
const int PIN_ANALOG_C_HALL = A2;




//const int PIN_ANALOG_A_OPTOCOUPLE = A0;
//const int PIN_ANALOG_B_OPTOCOUPLE = A1;
//const int PIN_ANALOG_C_OPTOCOUPLE = A2;

//const int PIN_CONTROL_A_OPTOCOUPLE = 4;
//const int PIN_CONTROL_B_OPTOCOUPLE = 7;
//const int PIN_CONTROL_C_OPTOCOUPLE = 8;

//const byte OPTOCOUPLE_DELAY_US = 1; // us   fixme: need experiments
//int optocouple_threshold_level = 128; // fixme: need experiments


// BEMF sensors
//const int PIN_ANALOG_A_BEMF = A0;
//const int PIN_ANALOG_B_BEMF = A1;
//const int PIN_ANALOG_C_BEMF = A2;

// Current sensors
//const int PIN_ANALOG_A_CURRENT = A3;
//const int PIN_ANALOG_B_CURRENT = A4;
//const int PIN_ANALOG_C_CURRENT = A5;
const int PIN_ANALOG_ABC_CURRENT = A3;
int g_zero_abc_current = 0;
int g_abc_current = 0;

char g_algorithm = 'a'; // a - analog
                        // d - digital


const int DELAY_BETWEEN_STEP_US = 1; // us

const unsigned long PRINT_DT = 200;
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
	pinMode(PIN_PHASE_A_HI, OUTPUT);
	pinMode(PIN_PHASE_A_LO, OUTPUT);
	pinMode(PIN_PHASE_B_HI, OUTPUT);
	pinMode(PIN_PHASE_B_LO, OUTPUT);
	pinMode(PIN_PHASE_C_HI, OUTPUT);
	pinMode(PIN_PHASE_C_LO, OUTPUT);
	
	
	free_rotation();
	
	
	/*
	pinMode(PIN_CONTROL_A_OPTOCOUPLE, OUTPUT);
	pinMode(PIN_CONTROL_B_OPTOCOUPLE, OUTPUT);
	pinMode(PIN_CONTROL_C_OPTOCOUPLE, OUTPUT);
	*/


	/*
	  // fixme: acquire statistic in start - really need?

	int value_analog_a_hall = 0;
	int value_analog_b_hall = 0;
	int value_analog_c_hall = 0;

	sync_sensor_measurement(&value_analog_a_hall, &value_analog_b_hall, &value_analog_c_hall);
	g_hall_min = min(min(value_analog_a_hall, value_analog_b_hall), value_analog_c_hall);
	g_hall_max = max(max(value_analog_a_hall, value_analog_b_hall), value_analog_c_hall);
	g_hall_zero = (g_hall_min + g_hall_max)/2;
	sensor_statistic(1.0, VALUE_ERR, value_analog_a_hall, value_analog_b_hall, value_analog_c_hall);

	int i = 0;
	while (millis()<1000){ // delay(1000);
		sync_sensor_measurement(&value_analog_a_hall, &value_analog_b_hall, &value_analog_c_hall);
		sensor_statistic(i++, VALUE_ERR, value_analog_a_hall, value_analog_b_hall, value_analog_c_hall);
	}
	*/

	Serial.begin(115200);

	g_zero_abc_current = read_abc_current();
}


void loop()
{

	float old_analog_angle = 0.0;
	byte old_digital_angle = 0;

	unsigned long halfturn_timer_us = 0;


	int value_analog_a_hall = 0;
	int value_analog_b_hall = 0;
	int value_analog_c_hall = 0;

	// hall max min level
	int hall_max  = ADC_MIN;
	int hall_min  = ADC_MAX;
	int hall_zero = (ADC_MAX + ADC_MIN)/2;


	long int turn_counter = 0;
	unsigned long old_turn_timer_us = 0;



	while (true) {
		unsigned long current_time_ms = millis();
		unsigned long current_time_us = micros();
		halfturn_timer_us = current_time_us; // fixme
		
		sync_sensor_measurement(&value_analog_a_hall, &value_analog_b_hall, &value_analog_c_hall); // need return three value_analog_X_hall, and current
		sensor_statistic(STATISTIC_MEAN, VALUE_ERR, value_analog_a_hall, value_analog_b_hall, value_analog_c_hall, &hall_min, &hall_max, &hall_zero);
		
		int velocity = apply_pid(halfturn_timer_us, old_turn_timer_us);
		
		byte  digital_angle;
		float analog_angle;
		if (g_algorithm == 'd'){
			digital_angle = digital_read_angle(value_analog_a_hall, value_analog_b_hall, value_analog_c_hall, hall_zero);
			//analog_angle = analog_read_angle(value_analog_a_hall, value_analog_b_hall, value_analog_c_hall, hall_min, hall_max);
			turn_digital(abs(velocity), digital_angle, sign(velocity));
			turn_digital_statistic(digital_angle, old_digital_angle, &turn_counter, current_time_us, &old_turn_timer_us);
		} else {
			//digital_angle = digital_read_angle(value_analog_a_hall, value_analog_b_hall, value_analog_c_hall, hall_zero);
			analog_angle = analog_read_angle(value_analog_a_hall, value_analog_b_hall, value_analog_c_hall, hall_min, hall_max);
			turn_analog(abs(velocity), analog_angle, calculate_analog_angle_shift_by_velocity(velocity));
			turn_analog_statistic(analog_angle, old_analog_angle, &turn_counter, current_time_us, &old_turn_timer_us);
		}
		
		
		
		if (current_time_ms > g_time_to_print){
			g_time_to_print = current_time_ms + PRINT_DT;
			
			
			char buffer [50];
			
			sprintf (buffer, "current = %d\t", read_abc_current());
			Serial.print(buffer);
			
			//find_best_angle_shift();
			
			//g_analog_abc_shift_cw = fmap((float)analogRead(A4), 0.0, 1023.0, 0.0, 6.3);
			//Serial.print("angle_shift = "); Serial.print(g_analog_abc_shift_cw); Serial.print("\t"); // only for cw (velocity > 0)
			
			read_ctrl();
			
			Serial.print("rpm = "); Serial.print(get_rpm(halfturn_timer_us, old_turn_timer_us));
			
			Serial.print("\tanalog_angle = ");Serial.print(analog_angle);
			Serial.print("\tdigital_angle = ");Serial.print(digital_angle);
			
			//Serial.print("old_turn_time = "); Serial.print(old_turn_timer_us);
			//Serial.print("\tturn_time = "); Serial.print((turn_timer_us - old_turn_timer_us)/1000);
			//Serial.print("\thalf_turn_time = "); Serial.print((halfturn_timer_us - old_turn_timer_us)/1000);
			//Serial.print("\tmax_turn_time = "); Serial.print((max(g_turn_timer_us - old_turn_timer_us, halfturn_timer_us - old_turn_timer_us))/1000);
			
			
			//Serial.print("\told_angle = "); Serial.print(old_analog_angle);
			//Serial.print("\tangle = "); Serial.print(analog_read_angle());
			
			
			//Serial.print("\tturn_counter = "); Serial.print(turn_counter);
			
			//Serial.print("\ta_hall = "); Serial.print(a_hall_value);
			//Serial.print("\tb_hall = "); Serial.print(b_hall_value);
			//Serial.print("\tc_hall = "); Serial.print(c_hall_value);
			Serial.print("\tmax = "); Serial.print(hall_max);
			Serial.print("\tmin = "); Serial.print(hall_min);
			//Serial.print("\tzero = "); Serial.print(hall_zero);
			
			Serial.println();
		}
		
		
		//Serial.println();
		//delay(200);



		if (g_algorithm == 'd'){
			old_digital_angle = digital_angle;
		} else {
			old_analog_angle = analog_angle;
		}
		
		delayMicroseconds(DELAY_BETWEEN_STEP_US);
	}
}
