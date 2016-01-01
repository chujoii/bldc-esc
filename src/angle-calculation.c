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

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

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
	float result_angle = 0.0;
	float tmp_diff;
	
	tmp_diff = cycle_max_diff_from_three(a1, b1, c1, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a1;
		printf("*");
	}
	printf("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a1, b1, c1, tmp_diff, min_angle_diff, result_angle);
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a1, b1, c2, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a1;
		printf("*");
	}
	printf("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a1, b1, c2, tmp_diff, min_angle_diff, result_angle);
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a1, b2, c1, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a1;
		printf("*");
	}
	printf("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a1, b2, c1, tmp_diff, min_angle_diff, result_angle);
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a1, b2, c2, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a1;
		printf("*");
	}
	printf("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a1, b2, c2, tmp_diff, min_angle_diff, result_angle);
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a2, b1, c1, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a2;
		printf("*");
	}
	printf("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a2, b1, c1, tmp_diff, min_angle_diff, result_angle);
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a2, b1, c2, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a2;
		printf("*");
	}
	printf("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a2, b1, c2, tmp_diff, min_angle_diff, result_angle);
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a2, b2, c1, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a2;
		printf("*");
	}
	printf("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a2, b2, c1, tmp_diff, min_angle_diff, result_angle);
	if (tmp_diff < epsilon) {return result_angle;}
	
	tmp_diff = cycle_max_diff_from_three(a2, b2, c2, point_of_cycle_max);
	if (tmp_diff < min_angle_diff){
		min_angle_diff = tmp_diff;
		result_angle = a2;
		printf("*");
	}
	printf("a=%7.2f\tb=%7.2f\tc=%7.2f\tdiff=%7.2f\tmin=%7.2f\ta=%7.2f\n", a2, b2, c2, tmp_diff, min_angle_diff, result_angle);
	return result_angle;
}

float calculation_angle_from_three_phases(float a, float b, float c) // int prev_angle, int prev_step)
{
	// a = sin(x +   0 + radomerror)
	// b = sin(x + 120 + radomerror)
	// c = sin(x + 240 + radomerror)
	// x = ?


	// all value in degree
	//
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
	float angle_a_1 = asin(a) * (180/3.14);// - angle_a_shift; // - 0
	float angle_b_1 = asin(b) * (180/3.14);// - angle_b_shift; // - 120
	float angle_c_1 = asin(c) * (180/3.14);// - angle_c_shift; // - 240
	printf("angle_a_1=%7.2f\tangle_b_1=%7.2f\tangle_c_1=%7.2f\n\n", angle_a_1, angle_b_1, angle_c_1);
	// angle_a_1= -50.38	angle_b_1=-129.79	angle_c_1=-173.04

	printf("a-0\tb-120\tc-240\n");
	angle_a_1 = angle_a_1 - angle_a_shift; // - 0
	angle_b_1 = angle_b_1 - angle_b_shift; // - 120
	angle_c_1 = angle_c_1 - angle_c_shift; // - 240
	printf("angle_a_1=%7.2f\tangle_b_1=%7.2f\tangle_c_1=%7.2f\n\n", angle_a_1, angle_b_1, angle_c_1);

	
        //
	// period of sin = 360[degree] = 2*3.14[rad]
	// 
	//
	printf("cycle constrain\n");
	angle_a_1 = cycle_constrain_angle(angle_a_1, 0.0, point_of_cycle_max);
	angle_b_1 = cycle_constrain_angle(angle_b_1, 0.0, point_of_cycle_max);
	angle_c_1 = cycle_constrain_angle(angle_c_1, 0.0, point_of_cycle_max);
	printf("angle_a_1=%7.2f\tangle_b_1=%7.2f\tangle_c_1=%7.2f\n\n", angle_a_1, angle_b_1, angle_c_1);
	// angle_a_1= 309.62	angle_b_1= 230.21	angle_c_1= 186.96
	//
	float angle_a_2;
	float angle_b_2;
	float angle_c_2;
	printf("symmetry\n");
	if (angle_a_1 < point_of_zero_cross_sin_x - angle_a_shift) {
		printf("a smaller\t\t");
		angle_a_2 = (point_of_symmetry_sin_x_plus  - angle_a_shift) * 2 - angle_a_1;
	} else {
		printf("a biger\t\t");
		angle_a_2 = (point_of_symmetry_sin_x_minus - angle_a_shift) * 2 - angle_a_1;
	}
	
	if (angle_b_1 < point_of_zero_cross_sin_x - angle_b_shift) {
		printf("b smaller\t\t");
		angle_b_2 = (point_of_symmetry_sin_x_plus  - angle_b_shift) * 2 - angle_b_1;
	} else {
		printf("b biger\t\t");
		angle_b_2 = (point_of_symmetry_sin_x_minus - angle_b_shift) * 2 - angle_b_1;
	}
	
	if (angle_c_1 < point_of_zero_cross_sin_x - angle_c_shift) {
		printf("c smaller\t\t");
		angle_c_2 = (point_of_symmetry_sin_x_plus  - angle_c_shift) * 2 - angle_c_1;
	} else {
		printf("c biger\t\t");
		angle_c_2 = (point_of_symmetry_sin_x_minus - angle_c_shift) * 2 - angle_c_1;
	}
	printf("%7.2f\n", cycle_constrain_angle(point_of_zero_cross_sin_x - angle_c_shift, 0.0, 360));
	printf("angle_a_2=%7.2f\tangle_b_2=%7.2f\tangle_c_2=%7.2f\n", angle_a_2, angle_b_2, angle_c_2);

	angle_a_2 = cycle_constrain_angle(angle_a_2, 0.0, point_of_cycle_max);
	angle_b_2 = cycle_constrain_angle(angle_b_2, 0.0, point_of_cycle_max);
	angle_c_2 = cycle_constrain_angle(angle_c_2, 0.0, point_of_cycle_max);
	printf("angle_a_2=%7.2f\tangle_b_2=%7.2f\tangle_c_2=%7.2f\n\n", angle_a_2, angle_b_2, angle_c_2);
	// fixme need change math function asin to table function
	// fixme float -> int

	// http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
	// The asin() function computes the principal value of the arc
	// sine of __x. The returned value is in the range [-pi/2,
	// pi/2] radians. A domain error occurs for arguments not in
	// the range [-1, +1].



	float result = examine_angles(angle_a_1, angle_a_2, angle_b_1, angle_b_2, angle_c_1, angle_c_2, epsilon);
	printf("result angle = %7.2f\n", result);
	return result;
}

