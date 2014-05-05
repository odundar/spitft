/*
Copyright (c) 2011 Arduino. All right reserved.
Copyright (c) 2013 Intel Corporation.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <Arduino.h>
#include <errno.h>
#include <interrupt.h>
#include <Mux.h>
#include <sysfs.h>
#include <trace.h>
#include "variant.h"

#define MY_TRACE_PREFIX "variant"

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////// Galileo Fab D ////////////////////////////

const int mux_sel_analog[NUM_ANALOG_INPUTS] = {
MUX_SEL_AD7298_VIN0,
MUX_SEL_AD7298_VIN1,
MUX_SEL_AD7298_VIN2,
MUX_SEL_AD7298_VIN3,
MUX_SEL_AD7298_VIN4,
MUX_SEL_AD7298_VIN5,
};

const int mux_sel_uart[NUM_UARTS][MUX_DEPTH_UART] = {
/* This is auto-indexed (board pinout) */
[0]	=	{MUX_SEL_NONE, MUX_SEL_NONE},	// ttyGS0 - USB not muxed
[1]	=	{MUX_SEL_UART0_RXD,	MUX_SEL_UART0_TXD},	// ttyS0 - muxed
[2] =	{MUX_SEL_NONE, MUX_SEL_NONE},	// ttyS1 - not muxed
};

const int mux_sel_spi[NUM_SPI][MUX_DEPTH_SPI] = {
[0] =	{
MUX_SEL_NONE,
MUX_SEL_NONE,
MUX_SEL_NONE
},
[1] =	{
MUX_SEL_SPI1_MOSI,
MUX_SEL_SPI1_MISO,
MUX_SEL_SPI1_SCK
},
};

/* Route pin2 to SoC */
mux_sel_t MuxDesc14[] = {
{31, LOW, FN_GPIO_OPENDRAIN},
};

/* Route pin3 to SoC */
mux_sel_t MuxDesc15[] = {
{30, LOW, FN_GPIO_OPENDRAIN},
};

// Describe muxes for pin gpiolib id 16
mux_sel_t MuxDesc16[] = {
//gpio, value, type
{42, HIGH, FN_GPIO_OPENDRAIN},
{42, LOW, FN_SPI},	// SPI chip-select
};

/* Route pin3 to Cypress */
mux_sel_t MuxDesc18[] = {
{30, HIGH, FN_GPIO_OPENDRAIN},
};

mux_sel_t MuxDesc25[] = {
//gpio, value, type
{43, HIGH, FN_GPIO_FIXED_OUTPUT},
{43, LOW, FN_SPI}
};

/* Route pin2 to Cypress */
mux_sel_t MuxDesc32[] = {
{31, HIGH, FN_GPIO_OPENDRAIN},
};

mux_sel_t MuxDesc38 [] = {
//gpio, value, type
{54, HIGH, FN_GPIO_OPENDRAIN},
{54, LOW, FN_SPI},
};

mux_sel_t MuxDesc39[] = {
//gpio, value, type
{55, HIGH, FN_GPIO_OPENDRAIN},
{55, LOW, FN_SPI},
};

mux_sel_t MuxDesc44[] = {
//gpio, value, type
{37, HIGH, FN_GPIO_OPENDRAIN},
{37, LOW, FN_ANALOG},
};

mux_sel_t MuxDesc45[] = {
//gpio, value, type
{36, HIGH, FN_GPIO_OPENDRAIN},
{36, LOW, FN_ANALOG},
};

mux_sel_t MuxDesc46[] = {
//gpio, value, type
{23, HIGH, FN_GPIO_OPENDRAIN},
{23, LOW, FN_ANALOG},
};

mux_sel_t MuxDesc47[] = {
//gpio, value, type
{22, HIGH, FN_GPIO_OPENDRAIN},
{22, LOW, FN_ANALOG},
};

mux_sel_t MuxDesc48[] = {
//gpio, value, type
{21, HIGH, FN_GPIO_OPENDRAIN},
{21, LOW, FN_ANALOG},
{29, HIGH, FN_ANALOG},
{29, HIGH, FN_GPIO_OPENDRAIN},
{29, LOW, FN_I2C},
};

mux_sel_t MuxDesc49[] = {
//gpio, value, type
{20, HIGH, FN_GPIO_OPENDRAIN},
{20, LOW, FN_ANALOG},
{29, HIGH, FN_ANALOG},
{29, HIGH, FN_GPIO_OPENDRAIN},
{29, LOW, FN_I2C},
};

mux_sel_t MuxDesc50[] = {
//gpio, value, type
{40, HIGH, FN_GPIO_OPENDRAIN},
{40, LOW, FN_UART},
};

mux_sel_t MuxDesc51[] = {
//gpio, value, type
{41, HIGH, FN_GPIO_OPENDRAIN},
{41, LOW, FN_UART},
};

PinDescription AltPin2 =
{ 14,	NONE,	GPIO_FAST_IO2,	2,	NONE,	LOW,	(mux_sel_t*)&MuxDesc14,	MUX_SIZE(MuxDesc14),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL };	// pin2 (when routed to SoC)

PinDescription AltPin3 =
{ 15,	NONE,	GPIO_FAST_IO3,	3,	NONE,	LOW,	(mux_sel_t*)&MuxDesc15,	MUX_SIZE(MuxDesc15),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL };	// pin3 (when routed to SoC)


// Sorted by Linux GPIO ID
PinDescription g_APinDescription[]=
{
// gpiolib alias fastid ardid Initial FixdSt ptMuxDesc, MuxCount type Handle Alt pAlt
{ 0,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_PULLUP,	-1, 0,	NULL	},	// pullup for GPIO14 (I02)
{ 1,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_PULLUP,	-1,	0,	NULL	},	// pullup for GPIO15 (IO3)
{ 2,	NONE,	NONE,	NONE,	NONE,	NONE,	NULL,	0,	FN_RESERVED,	-1,	0,	NULL	},	// S3 resume input - don't touch
{ 3,	NONE,	NONE,	20,	NONE,	NONE,	NULL,	0,	FN_GPIO_FIXED_OUTPUT,	-1,	0,	NULL	},	// general purpose LED (GP LED)
{ 4,	NONE,	NONE,	NONE,	NONE,	HIGH,	NULL,	0,	FN_PULLUP,	-1,	0,	NULL	},	// level shifter OE
{ 7,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_RESERVED,	-1,	0,	NULL	},	// jumper to select i2c address of GPIO expander
{ 12,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_RESERVED,	-1,	0,	NULL	},	// reset line to cypress - owned by Linux driver
{ 13,	NONE,	NONE,	NONE,	NONE,	NONE,	NULL,	0,	FN_RESERVED,	-1,	0,	NULL	},	// S0 interrupt on SOC GPIO<5>
{ 16,	NONE,	NONE,	10,	0,	NONE,	(mux_sel_t*)&MuxDesc16,	MUX_SIZE(MuxDesc16),	FN_GPIO_FIXED_OUTPUT,	-1,	0,	NULL	},	// (PWM7) selected by GPIO42_HIGH (IO10)
{ 17,	NONE,	NONE,	5,	NONE,	LOW,	NULL,	0,	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// (PWM5) (IO5)
{ 18,	NONE,	NONE,	3,	NONE,	LOW,	(mux_sel_t*)&MuxDesc18,	MUX_SIZE(MuxDesc18),	FN_GPIO_OPENDRAIN,	-1,	0,	&AltPin3 },	// (PWM3) selected by GPIO30_HIGH (IO3)
{ 19,	NONE,	NONE,	9,	NONE,	LOW,	NULL,	0,	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// (PWM1) (IO9)
{ 20,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux GPIO49 - AD7298:VIN5 (A5) (depends on GPIO29_HIGH)
{ 21,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux GPIO48 - AD7298:VIN4 (A4) (depends on GPIO29_HIGH)
{ 22,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux GPIO47 - AD7298:VIN3 (A3:FAST1)
{ 23,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux GPIO46 - AD7298:VIN2 (A2:FAST0)
{ 24,	NONE,	NONE,	6,	NONE,	LOW,	NULL,	0,	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// (PWM6) (IO6)
{ 25,	NONE,	NONE,	11,	0,	LOW,	(mux_sel_t*)&MuxDesc25,	MUX_SIZE(MuxDesc25),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// (PWM4) selected by GPIO43_HIGH (IO11)
{ 26,	NONE,	NONE,	8,	NONE,	LOW,	NULL,	0,	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// directly mapped (IO8)
{ 27,	NONE,	NONE,	7,	NONE,	LOW,	NULL,	0,	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// directly mapped (IO5)
{ 28,	NONE,	NONE,	4,	NONE,	LOW,	NULL,	0,	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// directly mapped (IO4)
{ 29,	NONE,	NONE,	NONE,	NONE,	HIGH,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux select I2C - VIN4/5 - GPIO48:49 (see Galileo datasheet)
{ 30,	NONE,	NONE,	NONE,	NONE,	HIGH,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// select IO3 routing: SoC or Cypress (default=HIGH=Cypress)
{ 31,	NONE,	NONE,	NONE,	NONE,	HIGH,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// select IO2 routing: SoC or Cypress (default=HIGH=Cypress)
{ 32,	NONE,	NONE,	2,	NONE,	LOW,	(mux_sel_t*)&MuxDesc32,	MUX_SIZE(MuxDesc32),	FN_GPIO_OPENDRAIN,	-1,	0,	&AltPin2 },	// selected by GPIO31_HIGH (IO2)
{ 36,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux VIN1 - GPIO45 (A1)
{ 37,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux VIN0 - GPIO44 (A0)
{ 38,	NONE,	NONE,	12,	0,	LOW,	(mux_sel_t*)&MuxDesc38,	MUX_SIZE(MuxDesc38),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// selected by GPIO54_HIGH (IO12)
{ 39,	20,	NONE,	13,	0,	LOW,	(mux_sel_t*)&MuxDesc39,	MUX_SIZE(MuxDesc39),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// selected by GPIO55_HIGH (IO13)
{ 40,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux GPIO50 - ttyS0_RX (IO0:RX)
{ 41,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux GPIO51 - ttyS0_TX (IO1:TX)
{ 42,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux GPIO16(PWM7) - SPI1_CS (IO10:SPI1_CS)
{ 43,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux GPIO25(PWM4) - SPI1_MOSI (IO11:SPI1_MOSI)
{ 44,	NONE,	NONE,	14,	0,	LOW,	(mux_sel_t*)&MuxDesc44,	MUX_SIZE(MuxDesc44),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// selected by GPIO37_HIGH (A0)
{ 45,	NONE,	NONE,	15,	0,	LOW,	(mux_sel_t*)&MuxDesc45,	MUX_SIZE(MuxDesc45),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// selected by GPIO36_HIGH (A1)
{ 46,	NONE,	NONE,	16,	0,	LOW,	(mux_sel_t*)&MuxDesc46,	MUX_SIZE(MuxDesc46),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// selected by GPIO23_HIGH (A2)
{ 47,	NONE,	NONE,	17,	0,	LOW,	(mux_sel_t*)&MuxDesc47,	MUX_SIZE(MuxDesc47),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// selected by GPIO22_HIGH (A3)
{ 48,	NONE,	NONE,	18,	0,	LOW,	(mux_sel_t*)&MuxDesc48,	MUX_SIZE(MuxDesc48),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// selected by GPIO29_HIGH & GPIO21_HIGH(A4:SDA)
{ 49,	NONE,	NONE,	19,	0,	LOW,	(mux_sel_t*)&MuxDesc49,	MUX_SIZE(MuxDesc49),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// selected by GPIO29_HIGH & GPIO20_HIGH(A5:SCL)
{ 50,	NONE,	NONE,	0,	1,	LOW,	(mux_sel_t*)&MuxDesc50,	MUX_SIZE(MuxDesc50),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// selected by GPIO40_HIGH (IO0)
{ 51,	NONE,	NONE,	1,	1,	LOW,	(mux_sel_t*)&MuxDesc51,	MUX_SIZE(MuxDesc51),	FN_GPIO_OPENDRAIN,	-1,	0,	NULL	},	// selected by GPIO41_HIGH (IO1)
{ 52,	NONE,	NONE,	NONE,	NONE,	NONE,	NULL,	0,	FN_RESERVED,	-1,	0,	NULL	},	// shield reset to s/w
{ 53,	NONE,	NONE,	NONE,	NONE,	NONE,	NULL,	0,	FN_RESERVED,	-1,	0,	NULL	},	// reset to h/w
{ 54,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux GPIO38 - SPI1_MISO (IO12:SPI1_MISO)
{ 55,	NONE,	NONE,	NONE,	NONE,	LOW,	NULL,	0,	FN_MUX,	-1,	0,	NULL	},	// mux GPIO39 - SPI1_SCK (IO13:SPI1_SCK)
};

uint32_t sizeof_g_APinDescription;

uint32_t ardPin2DescIdx[GPIO_TOTAL];

// Sorted by Linux PWM ID
PwmDescription g_APwmDescription[] = {
/* 0 is unused */
{ 1,	9,	-1,	-1 },
/* 2 is unused */
{ 3,	3,	-1,	-1 },
{ 4,	11,	-1,	-1 },
{ 5,	5,	-1,	-1 },
{ 6,	6,	-1,	-1 },
{ 7,	10,	-1,	-1 },
};
uint32_t sizeof_g_APwmDescription;

// Sorted Arduino Pin ID
PinState g_APinState[]=
{
/* uCurrentPwm uCurrentInput */
{ 0,	1 },	/* 0 */
{ 0,	1 },	/* 1 */
{ 0,	1 },	/* 2 */
{ 0,	1 },	/* 3 - PWM */
{ 0,	1 },	/* 4 */
{ 0,	1 },	/* 5 - PWM */
{ 0,	1 },	/* 6 - PWM */
{ 0,	1 },	/* 7 */
{ 0,	1 },	/* 8 */
{ 0,	1 },	/* 9 - PWM */
{ 0,	1 },	/* 10 - PWM */
{ 0,	1 },	/* 11 - PMW */
{ 0,	1 },	/* 12 */
{ 0,	1 },	/* 13 */
{ 0,	1 },	/* 14 */
{ 0,	1 },	/* 15 */
{ 0,	1 },	/* 16 */
{ 0,	1 },	/* 17 */
{ 0,	1 },	/* 18 */
{ 0,	1 },	/* 19 */
};

#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------

int variantPinMode(uint8_t pin, uint8_t mode)
{
/*
* Pin2 and pin3 can be individually assigned to a SoC or a Cypress
* GPIO.
*
* The pin at this time is always set as Cypress-handled.
*/

int ret = 0;
PinDescription *p = NULL;

if (pin >= GPIO_TOTAL){
trace_error("%s: invalid pin%u", __func__, pin);
return PIN_EINVAL;
}

/* Nothing to do if it's not pin2 nor pin3 */
if (2 != pin && 3 != pin) {
return 0;
}

/* Search for entry */
p = &g_APinDescription[ardPin2DescIdx[pin]];


/* Cypress is first entry: disable alternate */
p->iAlternate = 0;
trace_debug("%s: disable SoC GPIO for pin%u",
__func__, pin);

/* Reflect actual muxing change */
ret = muxSelectDigitalPin(pin);
if (ret) {
trace_error("%s: can't set mux for pin%d",
__func__, pin);
return ret;
}
return 0;
}

int variantPinModeIRQ(uint8_t pin, uint8_t mode)
{
/*
* Pin2 and pin3 can be individually assigned to a SoC or a Cypress
* GPIO.
*
* Assignment depends on the triggering mode:
* - CHANGE: Cypress
* - remaining: SoC
*/

PinDescription *p = NULL;
int ret = 0;

if (pin >= GPIO_TOTAL){
trace_error("%s: invalid pin%u", __func__, pin);
return PIN_EINVAL;
}

/* Nothing to do if it's not pin2 nor pin3 */
if (2 != pin && 3 != pin) {
return 0;
}

/* Search for entry */
p = &g_APinDescription[ardPin2DescIdx[pin]];

if (CHANGE != mode) {
/* SoC is second entry: enable alternate */
p->iAlternate = 1;
trace_debug("%s: enable SoC GPIO for pin%u",
__func__, pin);
} else {
/* Cypress is first entry: disable alternate */
p->iAlternate = 0;
trace_debug("%s: enable Cypress GPIO for pin%u", __func__,
pin);
}

/* Reflect actual muxing change */
ret = muxSelectDigitalPin(pin);
if (ret) {
trace_error("%s: can't set mux for pin%d",
__func__, pin);
return ret;
}

return 0;
}

void variantEnableFastGpio(int pin)
{
int entryno = ardPin2DescIdx[pin];
PinDescription *p = NULL;
int ret = 0;

if (entryno >= sizeof_g_APinDescription) {
trace_error("%s: ardPin2DescIdx[%d] == %d >= "
"sizeof_g_APinDescription", __func__, pin, entryno);
return;
}

/* Enable alternate to route to SoC */
p = &g_APinDescription[entryno];
p->iAlternate = 1;

/* Reflect actual muxing change */
ret = muxSelectDigitalPin(pin);
if (ret) {
trace_error("%s: can't set mux for pin%d",
__func__, pin);
}
}

void variantEnablePullup(uint8_t pin, int enable)
{
/*
* This is very FabD-specific: handle external pullups that are
* connected to SoC GPIOs on pin2 and pin3.
*
* This may not be the most brilliant solution. But hey, this
* external pullups are a hardware hack in the first place.
*/

int gpio = -1;

if (pin != 2 && pin != 3) {
return;
}

if (2 == pin) {
/* gpio0 is our pullup */
gpio = 0;
if (enable) {
sysfsGpioDirection(gpio, 1, HIGH);
} else {
sysfsGpioDirection(gpio, 1, LOW);
}
}
if (3 == pin) {
/* gpio1 is our pullup */
gpio = 1;
if (enable) {
sysfsGpioDirection(gpio, 1, HIGH);
} else {
sysfsGpioDirection(gpio, 1, LOW);
}
}
}

void __libc_init_array(void);

void init( int argc, char * argv[] )
{
sizeof_g_APinDescription = sizeof(g_APinDescription)/sizeof(struct _PinDescription);
pinInit();
}
