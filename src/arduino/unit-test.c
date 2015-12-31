#include <stdio.h>
#include <math.h>


float angle_a_shift =   0.0;  
float angle_b_shift = 120.0;
float angle_c_shift = 240.0;

const float point_of_symmetry_sin_x_plus  =  90.0;
const float point_of_symmetry_sin_x_minus = 270.0;
const float point_of_zero_cross_sin_x     = 180.0;

float cycle_constrain_angle (float x, float xmin, float xmax)
{
	while (x < xmin) {x = x + xmax;}
	while (x > xmax) {x = x - xmax;}
	return x;
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
	angle_a_1 = cycle_constrain_angle(angle_a_1, 0.0, 360.0);
	angle_b_1 = cycle_constrain_angle(angle_b_1, 0.0, 360.0);
	angle_c_1 = cycle_constrain_angle(angle_c_1, 0.0, 360.0);
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

	angle_a_2 = cycle_constrain_angle(angle_a_2, 0.0, 360.0);
	angle_b_2 = cycle_constrain_angle(angle_b_2, 0.0, 360.0);
	angle_c_2 = cycle_constrain_angle(angle_c_2, 0.0, 360.0);
	printf("angle_a_2=%7.2f\tangle_b_2=%7.2f\tangle_c_2=%7.2f\n\n", angle_a_2, angle_b_2, angle_c_2);
	// fixme need change math function asin to table function
	// fixme float -> int

	// http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
	// The asin() function computes the principal value of the arc
	// sine of __x. The returned value is in the range [-pi/2,
	// pi/2] radians. A domain error occurs for arguments not in
	// the range [-1, +1].



	//float diff_1 = 
	
	
	return (angle_a_1 + angle_b_1 + angle_c_1)/3.0;
}

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
