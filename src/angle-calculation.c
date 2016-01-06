/*
  angle-calculation.c --- calculate angle of motor rotor by information from magnetic (or optic) sensor

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

#ifdef DEBUG
#define DEBUG_PRINT(msg) printf msg
#else
#define DEBUG_PRINT(msg)
#endif


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


const unsigned char t_sin[256] = {0, 1, 3, 4, 6, 7, 9, 10, 12, 14, 15, 17, 18, 20, 21, 23, 25, 26, 28, 29, 31, 32, 34, 35, 37, 39, 40, 42, 43, 45, 46, 48, 49, 51, 52, 54, 56, 57, 59, 60, 62, 63, 65, 66, 68, 69, 71, 72, 74, 75, 77, 78, 80, 81, 83, 84, 86, 87, 89, 90, 92, 93, 94, 96, 97, 99, 100, 102, 103, 105, 106, 107, 109, 110, 112, 113, 115, 116, 117, 119, 120, 121, 123, 124, 126, 127, 128, 130, 131, 132, 134, 135, 136, 138, 139, 140, 142, 143, 144, 145, 147, 148, 149, 151, 152, 153, 154, 156, 157, 158, 159, 161, 162, 163, 164, 165, 167, 168, 169, 170, 171, 172, 174, 175, 176, 177, 178, 179, 180, 181, 182, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 208, 209, 210, 211, 212, 213, 214, 215, 215, 216, 217, 218, 219, 219, 220, 221, 222, 223, 223, 224, 225, 226, 226, 227, 228, 228, 229, 230, 230, 231, 232, 232, 233, 234, 234, 235, 235, 236, 237, 237, 238, 238, 239, 239, 240, 240, 241, 241, 242, 242, 243, 243, 244, 244, 245, 245, 246, 246, 246, 247, 247, 248, 248, 248, 249, 249, 249, 250, 250, 250, 250, 251, 251, 251, 251, 252, 252, 252, 252, 253, 253, 253, 253, 253, 253, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254};

const unsigned char t_asin[256] = {0, 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14, 15, 15, 16, 17, 17, 18, 19, 19, 20, 20, 21, 22, 22, 23, 24, 24, 25, 26, 26, 27, 28, 28, 29, 29, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 37, 37, 38, 39, 39, 40, 41, 41, 42, 43, 43, 44, 44, 45, 46, 46, 47, 48, 48, 49, 50, 50, 51, 52, 52, 53, 54, 54, 55, 56, 57, 57, 58, 59, 59, 60, 61, 61, 62, 63, 63, 64, 65, 65, 66, 67, 67, 68, 69, 70, 70, 71, 72, 72, 73, 74, 74, 75, 76, 77, 77, 78, 79, 79, 80, 81, 82, 82, 83, 84, 85, 85, 86, 87, 88, 88, 89, 90, 91, 91, 92, 93, 94, 94, 95, 96, 97, 97, 98, 99, 100, 100, 101, 102, 103, 104, 104, 105, 106, 107, 108, 108, 109, 110, 111, 112, 113, 113, 114, 115, 116, 117, 118, 118, 119, 120, 121, 122, 123, 124, 125, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 161, 162, 163, 164, 165, 167, 168, 169, 170, 172, 173, 174, 176, 177, 179, 180, 181, 183, 184, 186, 188, 189, 191, 192, 194, 196, 198, 200, 202, 204, 206, 208, 211, 213, 216, 218, 222, 225, 229, 233, 239, 254};


float cycle_constrain_angle (float x, float xmin, float xmax)
{
	while (x <  xmin) {x = x + xmax;}
	while (x >= xmax) {x = x - xmax;}
	return x;
}


float cycle_max_diff_from_three (float a, float b, float c, float cycle)
{
	float diffab = MIN(fabs(a - b), fabs(fabs(a - b) - cycle));
	float diffbc = MIN(fabs(b - c), fabs(fabs(b - c) - cycle));
	float diffca = MIN(fabs(c - a), fabs(fabs(c - a) - cycle));
	
	return MAX(MAX(diffab, diffbc), diffca);
}



float examine_angles(float a1, float a2, float b1, float b2, float c1, float c2, float epsilon)
{
	float min_angle_diff = point_of_cycle_max;
	float result_angle = point_of_cycle_min;
	float tmp_diff;
	
	tmp_diff = cycle_max_diff_from_three(a1, b1, c1, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a1;
		DEBUG_PRINT(("*"));
	}
	DEBUG_PRINT(("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a1, b1, c1, tmp_diff, min_angle_diff, result_angle));
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a1, b1, c2, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a1;
		DEBUG_PRINT(("*"));
	}
	DEBUG_PRINT(("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a1, b1, c2, tmp_diff, min_angle_diff, result_angle));
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a1, b2, c1, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a1;
		DEBUG_PRINT(("*"));
	}
	DEBUG_PRINT(("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a1, b2, c1, tmp_diff, min_angle_diff, result_angle));
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a1, b2, c2, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a1;
		DEBUG_PRINT(("*"));
	}
	DEBUG_PRINT(("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a1, b2, c2, tmp_diff, min_angle_diff, result_angle));
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a2, b1, c1, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a2;
		DEBUG_PRINT(("*"));
	}
	DEBUG_PRINT(("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a2, b1, c1, tmp_diff, min_angle_diff, result_angle));
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a2, b1, c2, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a2;
		DEBUG_PRINT(("*"));
	}
	DEBUG_PRINT(("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a2, b1, c2, tmp_diff, min_angle_diff, result_angle));
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a2, b2, c1, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a2;
		DEBUG_PRINT(("*"));
	}
	DEBUG_PRINT(("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a2, b2, c1, tmp_diff, min_angle_diff, result_angle));
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a2, b2, c2, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a2;
		DEBUG_PRINT(("*"));
	}
	DEBUG_PRINT(("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a2, b2, c2, tmp_diff, min_angle_diff, result_angle));
	return result_angle;
}

float calculation_angle_from_three_phases(float a, float b, float c) // int prev_angle, int prev_step)
{
	// a = sin(x +   0 + radomerror)
	// b = sin(x + 120 + radomerror)
	// c = sin(x + 240 + radomerror)
	// x = ?


	// all value in radian
	//
	// Each arc sine gives two solutions for the period.
	// for example real_motor_angle = 230 [degree] = 4 [rad]
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
	float angle_a_1 = asin(constrain(a, min_sin_val, max_sin_val));
	float angle_b_1 = asin(constrain(b, min_sin_val, max_sin_val));
	float angle_c_1 = asin(constrain(c, min_sin_val, max_sin_val));
	DEBUG_PRINT(("angle_a_1=%7.2f\tangle_b_1=%7.2f\tangle_c_1=%7.2f\n\n", angle_a_1, angle_b_1, angle_c_1));
	// angle_a_1= -50.38[degree]	angle_b_1=-129.79[degree]	angle_c_1=-173.04[degree]

	DEBUG_PRINT(("a-0\tb-120\tc-240\n"));
	angle_a_1 = angle_a_1 - angle_a_shift; // - 0[degree] = 0[rad]
	angle_b_1 = angle_b_1 - angle_b_shift; // - 120[degree] = pi*2/3[rad]
	angle_c_1 = angle_c_1 - angle_c_shift; // - 240[degree] = pi*4/3[rad]
	DEBUG_PRINT(("angle_a_1=%7.2f\tangle_b_1=%7.2f\tangle_c_1=%7.2f\n\n", angle_a_1, angle_b_1, angle_c_1));

	
        //
	// period of sin = 360[degree] = 2*3.14[rad]
	// 
	//
	DEBUG_PRINT(("cycle constrain\n"));
	angle_a_1 = cycle_constrain_angle(angle_a_1, point_of_cycle_min, point_of_cycle_max);
	angle_b_1 = cycle_constrain_angle(angle_b_1, point_of_cycle_min, point_of_cycle_max);
	angle_c_1 = cycle_constrain_angle(angle_c_1, point_of_cycle_min, point_of_cycle_max);
	DEBUG_PRINT(("angle_a_1=%7.2f\tangle_b_1=%7.2f\tangle_c_1=%7.2f\n\n", angle_a_1, angle_b_1, angle_c_1));
	// angle_a_1= 309.62[degree]	angle_b_1= 230.21[degree]	angle_c_1= 186.96[degree]
	//
	float angle_a_2;
	float angle_b_2;
	float angle_c_2;
	DEBUG_PRINT(("symmetry\n"));
	if (angle_a_1 < point_of_zero_cross_sin_x - angle_a_shift) {
		DEBUG_PRINT(("a smaller\t\t"));
		angle_a_2 = (point_of_symmetry_sin_x_plus  - angle_a_shift) * 2 - angle_a_1;
	} else {
		DEBUG_PRINT(("a biger\t\t"));
		angle_a_2 = (point_of_symmetry_sin_x_minus - angle_a_shift) * 2 - angle_a_1;
	}
	
	if (angle_b_1 < point_of_zero_cross_sin_x - angle_b_shift) {
		DEBUG_PRINT(("b smaller\t\t"));
		angle_b_2 = (point_of_symmetry_sin_x_plus  - angle_b_shift) * 2 - angle_b_1;
	} else {
		DEBUG_PRINT(("b biger\t\t"));
		angle_b_2 = (point_of_symmetry_sin_x_minus - angle_b_shift) * 2 - angle_b_1;
	}
	
	if (angle_c_1 < point_of_zero_cross_sin_x - angle_c_shift) {
		DEBUG_PRINT(("c smaller\t\t"));
		angle_c_2 = (point_of_symmetry_sin_x_plus  - angle_c_shift) * 2 - angle_c_1;
	} else {
		DEBUG_PRINT(("c biger\t\t"));
		angle_c_2 = (point_of_symmetry_sin_x_minus - angle_c_shift) * 2 - angle_c_1;
	}
	DEBUG_PRINT(("%7.2f\n", cycle_constrain_angle(point_of_zero_cross_sin_x - angle_c_shift, point_of_cycle_min, point_of_cycle_max)));
	DEBUG_PRINT(("angle_a_2=%7.2f\tangle_b_2=%7.2f\tangle_c_2=%7.2f\n", angle_a_2, angle_b_2, angle_c_2));

	angle_a_2 = cycle_constrain_angle(angle_a_2, point_of_cycle_min, point_of_cycle_max);
	angle_b_2 = cycle_constrain_angle(angle_b_2, point_of_cycle_min, point_of_cycle_max);
	angle_c_2 = cycle_constrain_angle(angle_c_2, point_of_cycle_min, point_of_cycle_max);
	DEBUG_PRINT(("angle_a_2=%7.2f\tangle_b_2=%7.2f\tangle_c_2=%7.2f\n\n", angle_a_2, angle_b_2, angle_c_2));
	// fixme need change math function asin to table function
	// fixme float -> int

	// http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
	// The asin() function computes the principal value of the arc
	// sine of __x. The returned value is in the range [-pi/2,
	// pi/2] radians. A domain error occurs for arguments not in
	// the range [-1, +1].



	float result = examine_angles(angle_a_1, angle_a_2, angle_b_1, angle_b_2, angle_c_1, angle_c_2, epsilon);
	DEBUG_PRINT(("result angle = %7.2f\n", result));
	return result;
}

