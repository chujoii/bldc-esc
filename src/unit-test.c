/*
  unit-test.c --- unit test for check bldc-esk functions

  Copyright (C) 2015 Roman V. Prikhodchenko



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

// for random rnd
#include <stdlib.h>
#include <time.h>


//#define DEBUG


float angle_a_shift =   0.0;  
float angle_b_shift = 120.0;
float angle_c_shift = 240.0;

const float point_of_symmetry_sin_x_plus  =  90.0;
const float point_of_symmetry_sin_x_minus = 270.0;
const float point_of_zero_cross_sin_x     = 180.0;
const float point_of_cycle_min            =   0.0;
const float point_of_cycle_max            = 360.0;

const float epsilon = 1.0;

const float test_epsilon = 6.0; // decrease for detect error in max/min values of sin(x+q) q=0,120,270: x+q=30, *90*, 150, 120, *270*, 330

const float dispersion = 0.01; // for "y" from sensor   y=sin(x)

const float min_sin_val = -1.0;
const float max_sin_val =  1.0;

float constrain (float x, float minx, float maxx);

#include "angle-calculation.c"

int g_count_err = 0;
int g_count_all = 0;


float frandom (float max_a)
{
	return (float)rand()/((float)RAND_MAX/max_a);
}

float constrain (float x, float minx, float maxx)
{
	if (x > maxx) {return maxx;}
	if (x < minx) {return minx;}
	return x;
}

int unit_test_calculation_angle_from_three_phases(float test_angle)
{
	printf("unit_test_calculation_angle_from_three_phases\t");
	printf("original_angle=%7.2f\t", test_angle);
	float result = calculation_angle_from_three_phases(sin((test_angle + angle_a_shift) * (3.14/180.0)),
							   sin((test_angle + angle_b_shift) * (3.14/180.0)),
							   sin((test_angle + angle_c_shift) * (3.14/180.0)));
	
	printf("calculated_angle=%7.2f\t", result);
	if (cycle_max_diff_from_three (test_angle, result, result, point_of_cycle_max) < test_epsilon) {
		printf("Ok\n");
		return 0;
	} else {
		printf("err\n");
		return 1;
	}
}

int unit_test_rnd_calculation_angle_from_three_phases(float test_angle, float disp)
{
	printf("unit_test_rnd_calculation_angle_from_three_phases\t");
	printf("original_angle=%7.2f\t", test_angle);
	float result = calculation_angle_from_three_phases(sin((test_angle + angle_a_shift) * (3.14/180.0)) + frandom(disp) - disp/2.0,
							   sin((test_angle + angle_b_shift) * (3.14/180.0)) + frandom(disp) - disp/2.0,
							   sin((test_angle + angle_c_shift) * (3.14/180.0)) + frandom(disp) - disp/2.0);
	
	printf("calculated_angle=%7.2f\t", result);
	if (cycle_max_diff_from_three (test_angle, result, result, point_of_cycle_max) < test_epsilon) {
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

	if (fabs(right_result - result) < test_epsilon) {
		printf("Ok\n");
		return 0;
	} else {
		printf("err\n");
		return 1;
	}
}

void counter(int i)
{
	g_count_err += i;
	g_count_all++;
}

int main()
{

	/* Intializes random number generator */
	time_t t;
	srand((unsigned) time(&t)); // fixme many start in one time == not unique numbers

   
	
	counter(unit_test_calculation_angle_from_three_phases(0));
	counter(unit_test_calculation_angle_from_three_phases(3));
	counter(unit_test_calculation_angle_from_three_phases(30));
	counter(unit_test_calculation_angle_from_three_phases(60));
	counter(unit_test_calculation_angle_from_three_phases(90));
	counter(unit_test_calculation_angle_from_three_phases(120));
	counter(unit_test_calculation_angle_from_three_phases(150));
	counter(unit_test_calculation_angle_from_three_phases(180));
	counter(unit_test_calculation_angle_from_three_phases(210));
	counter(unit_test_calculation_angle_from_three_phases(240));
	counter(unit_test_calculation_angle_from_three_phases(270));
	counter(unit_test_calculation_angle_from_three_phases(300));
	counter(unit_test_calculation_angle_from_three_phases(330));
	counter(unit_test_calculation_angle_from_three_phases(360));
	
	int i;
	for(i = 0; i < 1000; ++i)
	{
		counter(unit_test_rnd_calculation_angle_from_three_phases(0, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(3, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(30, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(60, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(90, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(120, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(150, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(180, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(210, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(240, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(270, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(300, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(330, dispersion));
		counter(unit_test_rnd_calculation_angle_from_three_phases(360, dispersion));
	}
	
	counter(unit_test_cycle_max_diff_from_three(10.0, 20.0, 30.0,       20.0)); // 20-10=10 30-20=10 *30-10=20* -> maxdiff = 20
	counter(unit_test_cycle_max_diff_from_three(30.0, 32.0, 330.0,      62.0)); // 32-30=2 330-32=min(62 298)=62 330-30=min(60 300)=60 -> maxdiff = 61
	


	if (g_count_err == 0){
		printf("all Ok\n");
		return 0;
	} else {
		printf("error in %d of %d test\n", g_count_err, g_count_all);
		return 1;
	}
}
