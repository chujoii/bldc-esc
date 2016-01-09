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


void read_ctrl()
{
	if (Serial.available() > 0) {
		byte incoming_byte;
                // read the incoming byte:
                incoming_byte = Serial.read();
		
		if ((incoming_byte != 0xa) && (g_cmd_line_length<g_cmd_line_max_length)) {
			g_cmd_line[g_cmd_line_length++] = incoming_byte;
		} else {
			exec_cmd(g_cmd_line, g_cmd_line_length - 1);
			//g_cmd_line = ""; // fixme: need reset command line for next command
			g_cmd_line_length = 0;
		}
	}
}

void exec_cmd(char *cmd, int cmd_len)
{
	// char + space = 2
	switch (cmd[0]) {
	case 'h': // help
	case 'H':
		Serial.print("h\t help\n");
		Serial.print("if print only one char, then printed current value of requested parameter\n");
		Serial.print("a 0.1\tangle in radian -6.3 .. 6.3\n");
		Serial.print("s 2\tvelocity -32768 .. 32767\n");
		Serial.print("S 3\tlimit speed 0 .. 32767\n");
		Serial.print("v 4\tvoltage (pwm) 0 .. 254\n");
		Serial.print("V 5\tlimit voltage (pwm) 0 .. 254\n");
		Serial.print("i 6\tcurrent -512 .. 512\n");
		Serial.print("I 7\tlimit current 0 .. 512\n");
		break;
	case 'a': // angle
		if (cmd_len > 2) {
			g_analog_abc_fine_tune_angle_shift = atof(&cmd[2]);
		} else {
			Serial.print("angle = ");
			// fixme: if ... g_analog_abc_shift_ccw
			Serial.print(g_analog_abc_shift_cw);
			Serial.print("(constanta) + ");
			Serial.print(g_analog_abc_fine_tune_angle_shift);
			Serial.println("(you shif)");
		}
		break;
	case 's': // velocity
		if (cmd_len > 2) {
		g_velocity_ctrl = atoi(&cmd[2]);
		} else {
			Serial.print("velocity = ");
			Serial.println(g_velocity_ctrl);
		}
		break;
	case 'S': // limit speed
		if (cmd_len > 2) {
		g_limit_speed_ctrl = atoi(&cmd[2]);
		} else {
			Serial.print("limit speed = ");
			Serial.println(g_limit_speed_ctrl);
		}
		break;
	case 'v': // voltage
		if (cmd_len > 2) {
		g_voltage_ctrl = atoi(&cmd[2]);
		} else {
			Serial.print("voltage = ");
			Serial.println(g_voltage_ctrl);
		}
		break;
	case 'V': // limit velocity
		if (cmd_len > 2) {
		g_limit_voltage_ctrl = atoi(&cmd[2]);
		} else {
			Serial.print("limit velocity = ");
			Serial.println(g_limit_voltage_ctrl);
		}
		break;
	case 'i': // current
		if (cmd_len > 2) {
		g_current_ctrl = atoi(&cmd[2]);
		} else {
			Serial.print("current = ");
			Serial.println(g_current_ctrl);
		}
		break;
	case 'I': // limit current
		if (cmd_len > 2) {
		g_limit_current_ctrl = atoi(&cmd[2]);
		} else {
			Serial.print("limit current = ");
			Serial.println(g_limit_current_ctrl);
		}
		break;
	default:
		Serial.println("error decode command, use 'h' for help");
	}
}
