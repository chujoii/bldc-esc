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

const float test_epsilon = 0.062; // (point_of_cycle_max - point_of_cycle_min) / 100.0

const float dispersion = 0.002; // (max_sin_val - min_sin_val) / 1000.0      for "y" from sensor   y=sin(x)


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
	float result = calculation_angle_from_three_phases(sin(test_angle + angle_a_shift),
							   sin(test_angle + angle_b_shift),
							   sin(test_angle + angle_c_shift));
	
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
	float result = calculation_angle_from_three_phases(sin(test_angle + angle_a_shift) + frandom(disp) - disp/2.0,
							   sin(test_angle + angle_b_shift) + frandom(disp) - disp/2.0,
							   sin(test_angle + angle_c_shift) + frandom(disp) - disp/2.0);
	
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


	counter(unit_test_calculation_angle_from_three_phases(4.01));         // 230[degree] see drawings/calculation_angle_from_three_phases.svg
	
	counter(unit_test_calculation_angle_from_three_phases(0));               // 0[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI/60));         // 3[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI/6.0));        // 30[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI/3.0));        // 60[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI/2.0));        // 90[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI * 2.0/3.0));  // 120[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI * 5.0/6.0));  // 150[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI));            // 180[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI * 7.0/6.0));  // 210[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI * 4.0/3.0));  // 240[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI * 3.0/2.0));  // 270[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI * 5.0/3.0));  // 300[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI * 11.0/6.0)); // 330[degree]
	counter(unit_test_calculation_angle_from_three_phases(M_PI * 2.0));      // 360[degree]
	
	int i;
	for(i = 0; i < 1000; ++i)
	{
		counter(unit_test_rnd_calculation_angle_from_three_phases(0, dispersion)); // 0[degree]


		counter(unit_test_rnd_calculation_angle_from_three_phases(0,               dispersion)); // 0[degree]  
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI/60,         dispersion)); // 3[degree]  
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI/6.0,        dispersion)); // 30[degree] 
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI/3.0,        dispersion)); // 60[degree] 
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI/2.0,        dispersion)); // 90[degree] 
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI * 2.0/3.0,  dispersion)); // 120[degree]
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI * 5.0/6.0,  dispersion)); // 150[degree]
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI,            dispersion)); // 180[degree]
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI * 7.0/6.0,  dispersion)); // 210[degree]
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI * 4.0/3.0,  dispersion)); // 240[degree]
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI * 3.0/2.0,  dispersion)); // 270[degree]
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI * 5.0/3.0,  dispersion)); // 300[degree]
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI * 11.0/6.0, dispersion)); // 330[degree]
		counter(unit_test_rnd_calculation_angle_from_three_phases(M_PI * 2.0,      dispersion)); // 360[degree]
	}

	counter(unit_test_cycle_max_diff_from_three(0.1, 0.2, 0.3,      0.2)); // 0.2-0.1=0.1   0.3-0.2=0.1   0.3-0.1=0.2 -> maxdiff = 0.2
	counter(unit_test_cycle_max_diff_from_three(0.1, 0.2, 6.2,      0.283185)); // 0.2-0.1=0.1   6.2-0.1=min(0.183185 6.1)=0.183185   6.2-0.2=min(0.283185 6.0)=0.283185 -> maxdiff = 0.283185
	


	if (g_count_err == 0){
		printf("all Ok\n");
		return 0;
	} else {
		printf("error in %d of %d test\n", g_count_err, g_count_all);
		return 1;
	}
}
