/*
  motor.ino --- program for write control action to bldc-esc

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


void search_phases_sensor_pinout(int speed, unsigned long waiting_time, int change_limit)
{
	// function only seach pinout
	// not apply finded pins
	
	int plausible_pin_analog_a_hall;
	int plausible_pin_analog_b_hall;
	int plausible_pin_analog_c_hall;
	
	do {
		plausible_pin_analog_a_hall = search_pinout(speed, waiting_time, change_limit, 'a');
		sprintf (buffer, "for phase A sensor pin = %d", plausible_pin_analog_a_hall);
		Serial.println(buffer);
		
		plausible_pin_analog_b_hall = search_pinout(speed, waiting_time, change_limit, 'b');
		sprintf (buffer, "for phase B sensor pin = %d", plausible_pin_analog_b_hall);
		Serial.println(buffer);
		
		plausible_pin_analog_c_hall = search_pinout(speed, waiting_time, change_limit, 'c');
		sprintf (buffer, "for phase C sensor pin = %d", plausible_pin_analog_c_hall);
		Serial.println(buffer);
	} while ((plausible_pin_analog_a_hall == plausible_pin_analog_b_hall) || (plausible_pin_analog_b_hall == plausible_pin_analog_c_hall) || (plausible_pin_analog_c_hall == plausible_pin_analog_a_hall));
}


int search_pinout(int speed, unsigned long waiting_time, int change_limit, char test_phase)
{
	int phase_num_1, phase_num_2;
	
	int old_x = analog_read_hall_sensor(PIN_ANALOG_A_HALL);
	int old_y = analog_read_hall_sensor(PIN_ANALOG_B_HALL);
	int old_z = analog_read_hall_sensor(PIN_ANALOG_C_HALL);

	
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

	unsigned long delay_time = waiting_time;
	unsigned long timer = millis() + waiting_time;
	
	// wait for stop changes
	while((abs(x - old_x) + abs(y - old_y) + abs(z - old_z) > change_limit) && (delay_time > 0)){
		if (one_or_two) {
			turn_digital(abs(speed), phase_num_1, sign(speed));
			one_or_two = !one_or_two;
		} else {
			turn_digital(abs(speed), phase_num_2, sign(speed));
			one_or_two = !one_or_two;
		}
		old_x = x;
		old_y = y;
		old_z = z;
		delay(delay_time);

		sync_sensor_measurement();
		sensor_statistic(STATISTIC_MEAN, VALUE_ERR);

		x = analog_read_hall_sensor(PIN_ANALOG_A_HALL);
		y = analog_read_hall_sensor(PIN_ANALOG_B_HALL);
		z = analog_read_hall_sensor(PIN_ANALOG_C_HALL);
		//sprintf (buffer, "x=%d\ty=%d\tz=%d\tdiff=%d", x, y, z, abs(x - old_x) + abs(y - old_y) + abs(z - old_z));
		//Serial.println(buffer);
		delay(delay_time);

		if (millis() > timer){
			delay_time--;
			timer = millis() + waiting_time;
		}
	}

	Serial.println(delay_time);
	
	if (x > y && x > z) {return PIN_ANALOG_A_HALL;}
	if (y > x && y > z) {return PIN_ANALOG_B_HALL;}
	if (z > x && z > y) {return PIN_ANALOG_C_HALL;}
	return PIN_ANALOG_C_HALL; // strange
}


void free_rotation()
{
	digitalWrite(PIN_PHASE_A_HI, LOW);
	digitalWrite(PIN_PHASE_A_LO, LOW);
	digitalWrite(PIN_PHASE_B_HI, LOW);
	digitalWrite(PIN_PHASE_B_LO, LOW);
	digitalWrite(PIN_PHASE_C_HI, LOW);
	digitalWrite(PIN_PHASE_C_LO, LOW);
}

void turn_analog(int speed, float angle, float angle_shift)
{
	// speed = scalar absolute value (magnitude) of velocity
	
	
	float phase_a_val = sin(angle + SHIFT_ANGLE_A + angle_shift);
	float phase_b_val = sin(angle + SHIFT_ANGLE_B + angle_shift);
	float phase_c_val = sin(angle + SHIFT_ANGLE_C + angle_shift);

	if (phase_a_val>0.0) {
		digitalWrite(PIN_PHASE_A_LO, LOW);
		analogWrite (PIN_PHASE_A_HI, (int)fmap(abs(phase_a_val), 0.0, 1.0, DAC_MIN, speed));
		//if (phase_a_val<0.5){DEBUGA_PRINT("(-");}else{DEBUGA_PRINT("(A");}
		//DEBUGA_PRINT((int)fmap(abs(phase_a_val), 0.0, 1.0, DAC_MIN, speed));DEBUGA_PRINT("\t");
	} else {
		digitalWrite(PIN_PHASE_A_HI, LOW);
		analogWrite (PIN_PHASE_A_LO, (int)fmap(abs(phase_a_val), 0.0, 1.0, DAC_MIN, speed));
		//if (phase_a_val>0.5){DEBUGA_PRINT("(-");}else{DEBUGA_PRINT("(a");}
		//DEBUGA_PRINT((int)fmap(abs(phase_a_val), 0.0, 1.0, DAC_MIN, speed));DEBUGA_PRINT("\t");
	}

	if (phase_b_val>0.0) {
		digitalWrite(PIN_PHASE_B_LO, LOW);
		analogWrite (PIN_PHASE_B_HI, (int)fmap(abs(phase_b_val), 0.0, 1.0, DAC_MIN, speed));
		//if (phase_b_val<0.5){DEBUGA_PRINT("-");}else{DEBUGA_PRINT("B");}
		//DEBUGA_PRINT((int)fmap(abs(phase_b_val), 0.0, 1.0, DAC_MIN, speed));DEBUGA_PRINT("\t");
	} else {
		digitalWrite(PIN_PHASE_B_HI, LOW);
		analogWrite (PIN_PHASE_B_LO, (int)fmap(abs(phase_b_val), 0.0, 1.0, DAC_MIN, speed));
		//if (phase_b_val>0.5){DEBUGA_PRINT("-");}else{DEBUGA_PRINT("b");}
		//DEBUGA_PRINT((int)fmap(abs(phase_b_val), 0.0, 1.0, DAC_MIN, speed));DEBUGA_PRINT("\t");
	}
	
	if (phase_c_val>0.0) {
		digitalWrite(PIN_PHASE_C_LO, LOW);
		analogWrite (PIN_PHASE_C_HI, (int)fmap(abs(phase_c_val), 0.0, 1.0, DAC_MIN, speed));
		//if (phase_c_val<0.5){DEBUGA_PRINT("-)\t");}else{DEBUGA_PRINT("C)\t");}
		//DEBUGA_PRINT((int)fmap(abs(phase_c_val), 0.0, 1.0, DAC_MIN, speed));DEBUGA_PRINT("\t");
	} else {
		digitalWrite(PIN_PHASE_C_HI, LOW);
		analogWrite (PIN_PHASE_C_LO, (int)fmap(abs(phase_c_val), 0.0, 1.0, DAC_MIN, speed));
		//if (phase_c_val>0.5){DEBUGA_PRINT("-)\t");}else{DEBUGA_PRINT("c)\t");}
		//DEBUGA_PRINT((int)fmap(abs(phase_c_val), 0.0, 1.0, DAC_MIN, speed));DEBUGA_PRINT("\t");
	}

	// fixme: need minimum 3 measurement by each turn:
	// what happend if number of measurement less than 3?
	// rigth answer motor stop because control become incorrect
	//
	//  g_old_analog_angle < 120[degree]     &&  angle > 240[degree]
	if (g_old_analog_angle < SHIFT_ANGLE_B   &&  angle > SHIFT_ANGLE_C){
		g_turn_counter++;
		g_old_turn_timer_us = g_turn_timer_us;
		g_turn_timer_us = micros();
		g_real_direction = 1;
	}
	//
	//  g_old_analog_angle > 240[degree]     &&  angle < 120[degree]
	if (g_old_analog_angle > SHIFT_ANGLE_C   &&  angle < SHIFT_ANGLE_B){
		g_turn_counter--;
		g_old_turn_timer_us = g_turn_timer_us;
		g_turn_timer_us = micros();
		g_real_direction = -1;
	}

	g_halfturn_timer_us = micros();
	
	g_old_analog_angle = angle;

}




void turn_digital(byte speed, byte digital_angle, boolean angle_shift)
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
	
	// speed = scalar absolute value (magnitude) of velocity
	

	
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


	if (angle_shift){
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
		analogWrite(PIN_PHASE_A_HI, speed);
		digitalWrite(PIN_PHASE_A_LO, LOW);
		analogWrite(PIN_PHASE_B_HI, 0);
		digitalWrite(PIN_PHASE_B_LO, HIGH);
		analogWrite(PIN_PHASE_C_HI, 0);
		digitalWrite(PIN_PHASE_C_LO, LOW);
		break;
	case B001:
		// a-hi c-lo
		DEBUGA_PRINT("[A-c]\t");
		analogWrite(PIN_PHASE_A_HI, speed);
		digitalWrite(PIN_PHASE_A_LO, LOW);
		analogWrite(PIN_PHASE_B_HI, 0);
		digitalWrite(PIN_PHASE_B_LO, LOW);
		analogWrite(PIN_PHASE_C_HI, 0);
		digitalWrite(PIN_PHASE_C_LO, HIGH);
		break;
	case B011:
		// b-hi c-lo
		DEBUGA_PRINT("[-Bc]\t");
		analogWrite(PIN_PHASE_A_HI, 0);
		digitalWrite(PIN_PHASE_A_LO, LOW);
		analogWrite(PIN_PHASE_B_HI, speed);
		digitalWrite(PIN_PHASE_B_LO, LOW);
		analogWrite(PIN_PHASE_C_HI, 0);
		digitalWrite(PIN_PHASE_C_LO, HIGH);
		break;
	case B010:
		// b-hi a-lo
		DEBUGA_PRINT("[aB-]\t");
		analogWrite(PIN_PHASE_A_HI, 0);
		digitalWrite(PIN_PHASE_A_LO, HIGH);
		analogWrite(PIN_PHASE_B_HI, speed);
		digitalWrite(PIN_PHASE_B_LO, LOW);
		analogWrite(PIN_PHASE_C_HI, 0);
		digitalWrite(PIN_PHASE_C_LO, LOW);
		break;
	case B110:
		// c-hi a-lo
		DEBUGA_PRINT("[a-C]\t");
		analogWrite(PIN_PHASE_A_HI, 0);
		digitalWrite(PIN_PHASE_A_LO, HIGH);
		analogWrite(PIN_PHASE_B_HI, 0);
		digitalWrite(PIN_PHASE_B_LO, LOW);
		analogWrite(PIN_PHASE_C_HI, speed);
		digitalWrite(PIN_PHASE_C_LO, LOW);
		break;
	case B100:
		// c-hi b-lo
		DEBUGA_PRINT("[-bC]\t");
		analogWrite(PIN_PHASE_A_HI, 0);
		digitalWrite(PIN_PHASE_A_LO, LOW);
		analogWrite(PIN_PHASE_B_HI, 0);
		digitalWrite(PIN_PHASE_B_LO, HIGH);
		analogWrite(PIN_PHASE_C_HI, speed);
		digitalWrite(PIN_PHASE_C_LO, LOW);
		break;
	default:
		// 000 111 error
		// fixme
		DEBUGA_PRINT("[---]\t");
		analogWrite(PIN_PHASE_A_HI, 0);
		digitalWrite(PIN_PHASE_A_LO, LOW);
		analogWrite(PIN_PHASE_B_HI, 0);
		digitalWrite(PIN_PHASE_B_LO, LOW);
		analogWrite(PIN_PHASE_C_HI, 0);
		digitalWrite(PIN_PHASE_C_LO, LOW);
	}

	if (g_old_digital_angle == B101 && digital_angle == B100){
		g_turn_counter++;
		g_old_turn_timer_us = g_turn_timer_us;
		g_turn_timer_us = micros();
		g_real_direction = 1;
	}
	if (g_old_digital_angle == B100 && digital_angle == B101){
		g_turn_counter--;
		g_old_turn_timer_us = g_turn_timer_us;
		g_turn_timer_us = micros();
		g_real_direction = -1;
	}
	
	g_halfturn_timer_us = micros();
	
	g_old_digital_angle = digital_angle;
}


void find_best_angle_shift(){
		if (abs(g_turn_counter - g_old_turn_counter) > g_best_turn_counter) {
			g_best_turn_counter = abs(g_turn_counter - g_old_turn_counter);
			g_best_angle_abc_shift = g_analog_abc_shift_cw;
		}
		

		g_analog_abc_shift_cw = g_analog_abc_shift_cw + 0.01;

		
		//DEBUGA_PRINT("angle_shift = "); DEBUGA_PRINT(g_analog_abc_shift_cw); DEBUGA_PRINT("\t"); // strange sprintf not work

		//DEBUGA_PRINT("best_angle_shift = "); DEBUGA_PRINT(g_best_angle_abc_shift); DEBUGA_PRINT("\t"); // strange sprintf not work

		//sprintf (buffer, "turn_diff = %d\t", abs(g_turn_counter - g_old_turn_counter));
		//DEBUGA_PRINT(buffer);
		
		//sprintf (buffer, "best_turn = %d\t", g_best_turn_counter);
		//DEBUGA_PRINT(buffer);


		//sprintf (buffer, "turn = %d\t", g_turn_counter);
		//DEBUGA_PRINT(buffer);



		// simple 2 column
		DEBUGA_PRINT(g_analog_abc_shift_cw); DEBUGA_PRINT("\t"); // strange sprintf not work
		sprintf (buffer, "%ld", g_turn_counter - g_old_turn_counter);
		DEBUGA_PRINT(buffer);
		
		
		g_old_turn_counter = g_turn_counter;
}
