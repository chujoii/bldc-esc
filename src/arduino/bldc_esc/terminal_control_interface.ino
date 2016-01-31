/*
  bldc_control_interface.ino --- control program for bldc-esc

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

 Version 0.1 was created at 2016.jan.08



 Code:
*/


void read_ctrl(char *algorithm)
{
	if (Serial.available() > 0) {
		byte incoming_byte;
                // read the incoming byte:
                incoming_byte = Serial.read();
		
		if ((incoming_byte != 0xa) && (g_cmd_line_length<g_cmd_line_max_length)) {
			g_cmd_line[g_cmd_line_length++] = incoming_byte;
		} else {
			exec_cmd(g_cmd_line, g_cmd_line_length - 1, &(*algorithm));
			for (int i = 0; i < g_cmd_line_max_length; i++){ // fixme: maybe need reset command line only from 0 to g_cmd_line_length
				g_cmd_line[i]='\0';
			}
			g_cmd_line_length = 0;
		}
	}
}




void exec_cmd(char *cmd, int cmd_len, char *algorithm)
{
	int i; // counter

	Serial.print("cmd_len = "); Serial.println(cmd_len);
	// char + space = 2
	switch (cmd[0]) {
	case 'h': // help
	case 'H':
		Serial.print("h\t help\n");
		Serial.print("if print only one char, then current value of requested parameter will be printed\n");
		Serial.print("a 0.1\tangle shift for cw  in radian -6.3 .. 6.3\n");
		Serial.print("A 0.1\tangle shift for ccw in radian -6.3 .. 6.3\n");
		Serial.print("s 2\tvelocity -32768 .. 32767\n");
		Serial.print("S 3\tlimit speed 0 .. 32767\n");
		Serial.print("u 4\tvoltage (pwm) 0 .. 254\n");
		Serial.print("U 5\tlimit voltage (pwm) 0 .. 254\n");
		Serial.print("i 6\tcurrent -512 .. 512\n");
		Serial.print("I 7\tlimit current 0 .. 512\n");
		Serial.print("g a\talgorithm: a - analog, d - digital\n");
		Serial.print("f 10 30 2\tsearch sensor pinout: speed, waiting_time[ms], change_limit\n");
		Serial.print("x 0.8 0.9 1.0\tproportional integral derivative coefficient for active parameter\n");
		break;
	case 'a': // angle cw
		if (cmd_len > 1) {
			g_analog_abc_shift_cw = atof(&cmd[2]);
		} else {
			Serial.print("angle shift = ");
			Serial.println(g_analog_abc_shift_cw);
		}
		break;
	case 'A': // angle ccw
		if (cmd_len > 1) {
			g_analog_abc_shift_ccw = atof(&cmd[2]);
		} else {
			Serial.print("angle shift = ");
			Serial.println(g_analog_abc_shift_ccw);
		}
		break;
	case 's': // velocity
		if (cmd_len > 1) {
			g_velocity_ctrl = atoi(&cmd[2]);
			g_main_ctrl_parameter = 's';
		} else {
			Serial.print("velocity = ");
			Serial.println(g_velocity_ctrl);
		}
		break;
	case 'S': // limit speed
		if (cmd_len > 1) {
		g_limit_speed_ctrl = atoi(&cmd[2]);
		} else {
			Serial.print("limit speed = ");
			Serial.println(g_limit_speed_ctrl);
		}
		break;
	case 'u': // voltage
		if (cmd_len > 1) {
			g_voltage_ctrl = atoi(&cmd[2]);
			g_main_ctrl_parameter = 'u';
		} else {
			Serial.print("voltage = ");
			Serial.println(g_voltage_ctrl);
		}
		break;
	case 'U': // limit voltage
		if (cmd_len > 1) {
			g_limit_voltage_ctrl = atoi(&cmd[2]);
		} else {
			Serial.print("limit velocity = ");
			Serial.println(g_limit_voltage_ctrl);
		}
		break;
	case 'i': // current
		if (cmd_len > 1) {
			g_current_ctrl = atoi(&cmd[2]);
			g_main_ctrl_parameter = 'i';
		} else {
			Serial.print("current = ");
			Serial.println(g_current_ctrl);
		}
		break;
	case 'I': // limit current
		if (cmd_len > 1) {
			g_limit_current_ctrl = atoi(&cmd[2]);
		} else {
			Serial.print("limit current = ");
			Serial.println(g_limit_current_ctrl);
		}
		break;
	case 'g': // algorithm: a - analog, d - digital
		if (cmd_len > 1) {
			if (cmd[2] == 'd') {*algorithm = 'd';}
			else               {*algorithm = 'a';}
		} else {
			Serial.print("algorithm = ");Serial.print(*algorithm);Serial.print("\n");
		}
		break;
	case 'f': // search sensor pinout
		if (cmd_len > 1) {
			int speed;        // fixme what happend if atoi return 0? or speed set to 255 and motor or driver burn?
			int waiting_time; // fixme what happend if atoi return 0?
			int change_limit; // fixme what happend if atoi return 0?
			
			i = 2;
			
			speed = constrain(atoi(&cmd[i]), DAC_MIN, g_limit_speed_ctrl);
			
			while (cmd[i] != ' ' && i < cmd_len){i++;}
			waiting_time = constrain(atoi(&cmd[i++]), 0, 20);
		
			while (cmd[i] != ' ' && i < cmd_len){i++;}
			change_limit = constrain(atoi(&cmd[i]), 1, ADC_MAX * 3);
			
			search_phases_sensor_pinout(speed, waiting_time, change_limit);
			free_rotation();
		} else {
			Serial.print("pin a hall = A");Serial.print(PIN_ANALOG_A_HALL - 14);
			Serial.print("\tpin b hall = A");Serial.print(PIN_ANALOG_B_HALL - 14);
			Serial.print("\tpin c hall = A");Serial.println(PIN_ANALOG_C_HALL - 14);
		}
		break;
	case 'x': // proportional integral derivative coefficient for active parameter
		if (cmd_len > 1) {
			float a,b,c;

			i = 2;
			
			a = atof(&cmd[i]);
			
			while (cmd[i] != ' ' && i < cmd_len){i++;}
			b = atof(&cmd[i++]);

			while (cmd[i] != ' ' && i < cmd_len){i++;}
			c = atof(&cmd[i]);

			switch (g_main_ctrl_parameter){
			case 's':
				g_velocity_ctrl_proportional = a;
				g_velocity_ctrl_integral     = b;
				g_velocity_ctrl_derivative   = c;
				break;
			case 'u':
				g_voltage_ctrl_proportional = a;
				g_voltage_ctrl_integral     = b;
				g_voltage_ctrl_derivative   = c;
				break;
			case 'i':
				g_current_ctrl_proportional = a;
				g_current_ctrl_integral     = b;
				g_current_ctrl_derivative   = c;
				break;
			}
		} else {
			switch (g_main_ctrl_parameter){
			case 's':
				Serial.print("for velocity");
				Serial.print("\tproportional = ");
				Serial.print(g_velocity_ctrl_proportional);
				Serial.print("\tintegral = ");
				Serial.print(g_velocity_ctrl_integral);
				Serial.print("\tderivative = ");
				Serial.println(g_velocity_ctrl_derivative);
				break;
			case 'u':
				Serial.print("for voltage");
				Serial.print("\tproportional = ");
				Serial.print(g_voltage_ctrl_proportional);
				Serial.print("\tintegral = ");
				Serial.print(g_voltage_ctrl_integral);
				Serial.print("\tderivative = ");
				Serial.println(g_voltage_ctrl_derivative);
				break;
			case 'i':
				Serial.print("for current");
				Serial.print("\tproportional = ");
				Serial.print(g_current_ctrl_proportional);
				Serial.print("\tintegral = ");
				Serial.print(g_current_ctrl_integral);
				Serial.print("\tderivative = ");
				Serial.println(g_current_ctrl_derivative);
				break;
			}
		}
		break;
	default:
		Serial.println("error decode command, use 'h' for help");
	}
}
