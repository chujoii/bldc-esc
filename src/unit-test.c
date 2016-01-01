/*
  unit-test.c --- unit test for check bldc-esk functions

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

 Code:
*/

#include <stdio.h>
#include <math.h>

float angle_a_shift =   0.0;  
float angle_b_shift = 120.0;
float angle_c_shift = 240.0;

const float point_of_symmetry_sin_x_plus  =  90.0;
const float point_of_symmetry_sin_x_minus = 270.0;
const float point_of_zero_cross_sin_x     = 180.0;
const float point_of_cycle_min            =   0.0;
const float point_of_cycle_max            = 360.0;

const float epsilon = 1.0;


#include "angle-calculation.c"

int unit_test_calculation_angle_from_three_phases(float test_angle)
{
	printf("unit_test_calculation_angle_from_three_phases\t");
	printf("original_angle=%7.2f\t", test_angle);
	float result = calculation_angle_from_three_phases(sin((test_angle + angle_a_shift) * (3.14/180.0)),
							   sin((test_angle + angle_b_shift) * (3.14/180.0)),
							   sin((test_angle + angle_c_shift) * (3.14/180.0)));
	
	if (fabs(test_angle - result) < epsilon) {
		printf("Ok\n");
		return 0;
	} else {
		printf("err\n");
		return 1;
	}
}

int unit_test_cycle_max_diff_from_three(float a, float b, float c, float right_result)
{
	printf("unit_test_cycle_max_diff_from_three\t");

	float result = cycle_max_diff_from_three(a, b, c, point_of_cycle_max);

	printf("a=%7.2f\tb=%7.2f\tb=%7.2f\tright_result=%7.2f\tcalculated_result=%7.2f\t", a, b, c, right_result, result);

	if (fabs(right_result - result) < epsilon) {
		printf("Ok\n");
		return 0;
	} else {
		printf("err\n");
		return 1;
	}
}


int main()
{
	int count = 0;
	count += unit_test_calculation_angle_from_three_phases(0);
	count += unit_test_calculation_angle_from_three_phases(3);
	count += unit_test_calculation_angle_from_three_phases(30);
	count += unit_test_calculation_angle_from_three_phases(60);
	count += unit_test_calculation_angle_from_three_phases(90);
	count += unit_test_calculation_angle_from_three_phases(120);
	count += unit_test_calculation_angle_from_three_phases(150);
	count += unit_test_calculation_angle_from_three_phases(180);
	count += unit_test_calculation_angle_from_three_phases(210);
	count += unit_test_calculation_angle_from_three_phases(240);
	count += unit_test_calculation_angle_from_three_phases(270);
	count += unit_test_calculation_angle_from_three_phases(300);
	count += unit_test_calculation_angle_from_three_phases(330);
	count += unit_test_calculation_angle_from_three_phases(360);

	count += unit_test_cycle_max_diff_from_three(10.0, 20.0, 30.0,       20.0); // 20-10=10 30-20=10 *30-10=20* -> maxdiff = 20
	count += unit_test_cycle_max_diff_from_three(30.0, 32.0, 330.0,      62.0); // 32-30=2 330-32=min(62 298)=62 330-30=min(60 300)=60 -> maxdiff = 61


	if (count == 0){
			printf("all Ok\n");
		return 0;
	} else {
		printf("error in test\n");
		return 1;
	}
}
