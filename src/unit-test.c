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

#include "angle-calculation.c"

int unit_test_calculation_angle_from_three_phases()
{
	//int result = 0;
	float test_angle = 230.0;
	printf("original_angle=%7.2f\n\n", test_angle);
	calculation_angle_from_three_phases(sin((test_angle + angle_a_shift) * (3.14/180.0)),
					    sin((test_angle + angle_b_shift) * (3.14/180.0)),
					    sin((test_angle + angle_c_shift) * (3.14/180.0)));//-0.77, -0.17, 0.92);
	return 0;
}

int main()
{
	unit_test_calculation_angle_from_three_phases();
	return 0;
}
