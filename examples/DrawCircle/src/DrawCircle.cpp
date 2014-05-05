/*
 * Copyright (C) Your copyright.
 *
 * Author: Onur Dundar
 * 
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <iostream>
#include "TFTv2.h"
#include "wiring_digital.h"
#include "variant.h"

using namespace std;

int main(int argc, char *argv[]) {
	init(argc, argv);
	fastGpioSCInit();

	cout << "Hello World" << endl; /* prints Hello World */
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(20, HIGH);
	cout << "LED ON" << endl;
	sleep(5);
	digitalWrite(LED_BUILTIN, LOW);
	cout << "LED OFF" << endl;
	/* Initialize SPI */
	//SPI.init();
	//SPI.begin();
	/*Initialize TFT */
	TFT_BL_ON;

	Tft.TFTinit();

	Tft.drawCircle(100, 100, 30, YELLOW); //center: (100, 100), r = 30 ,color : YELLOW

	Tft.drawCircle(100, 200, 40, CYAN); // center: (100, 200), r = 10 ,color : CYAN

	Tft.fillCircle(200, 100, 30, RED); //center: (200, 100), r = 30 ,color : RED

	Tft.fillCircle(200, 200, 30, BLUE); //center: (200, 200), r = 30 ,color : BLUE

	return 0;
}

