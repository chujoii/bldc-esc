/*
  generator-sin.c --- generate sinus table

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

 Code:
*/

#include <stdio.h>
#include <math.h>


void sin_tab (int max_x, int max_y)
{
	int x;

	printf("unsigned char T_SIN[256] = {");
	for (x=0; x<=max_x; x++){
		// generate sin(x); x=0..90[grad]
		// but              x=0..255[number]
		// x[grad] = x[number]*90/255
		// x[rad]  = x[grad] * (3.14/180)
		printf("0x%x", (unsigned char)(sin(x*90.0*3.14/(180.0*255.0)) * max_y));
		if (x<max_x){printf(", ");}
	}
	printf("};\n");
}



void asin_tab (int max_y, int max_x)
{
	int y;
	
	printf("unsigned char T_ASIN[256] = {");
	for (y=0; y<=max_y; y++){
		// generate asin(y); y=0..1[val]
		// but               y=0..255[number]
		// y[number] = y[number]*255
		//
		// result asin(y) in [rad]
		// x[grad] = x[rad]  * (180/3.14)
		//                        x=0..90[grad]
		// but x need in interval x=0..255[number]
		printf("0x%x", (unsigned char)(asin(((float)y)/((float)max_y))*180.0*max_x/(90.0*3.14)));
		if (y<max_y){printf(", ");}
	}
	printf("};\n");
}

int main (){
	sin_tab(255, 255);
	printf("\n");
	asin_tab(255, 254); // 254 because driver high and low
			    // transistors - ir2101 does not contains
			    // generator, and use pwm from high logic
			    // input for charge bootstrap capacitor
	return 0;
}
