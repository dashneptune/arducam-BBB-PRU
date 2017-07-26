 /*
 ============================================================================
 Name        : BBBCAM.c
 Author      : Lee
 Version     : V1.0
 Copyright   : ArduCAM demo (C)2015 Lee
 Description :

  Beaglebon Black library support for CMOS Image Sensor
  Copyright (C)2011-2015 ArduCAM.com. All right reserved

  Basic functionality of this library are based on the demo-code provided by
  ArduCAM.com. You can find the latest version of the library at
  http://www.ArduCAM.com

  Now supported controllers:
		-	OV7670
		-	MT9D111
		-	OV7675
		-	OV2640
		-	OV3640
		-	OV5642
		-	OV7660
		-	OV7725

	We will add support for many other sensors in next release.

  Supported MCU platform
 		-	Theoretically support all Arduino families
  	-	Arduino UNO R3				(Tested)
  	-	Arduino MEGA2560 R3		(Tested)
  	-	Arduino Leonardo R3		(Tested)
  	-	Arduino Nano					(Tested)
  	-	Arduino DUE						(Tested)
  	-	Arduino Yun						(Tested)  		
  	-	Beaglebon Black				(Tested)
  	-	Raspberry Pi					(Tested)
  	- ESP8266-12 						(Tested)

  If you make any modifications or improvements to the code, I would appreciate
  that you share the code with me so that I might include it in the next release.
  I can be contacted through http://www.ArduCAM.com

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*------------------------------------
	Revision History:
	2015/01/16  V1.0  by Lee  Inital library for Beaglebone Black.
	2015/10/12 	V1.1	by Lee	Optimization
--------------------------------------*/

#include "BBBCAM.h"
//#include "UTFT_SPI.h"
#include "memorysaver.h"
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <linux/i2c-dev.h>



/*
Function: 		BBBCAM
Param: 				camera model
Description :	BeagleBone Black Camera Instantiation,
							Initialize the Camera structure,
							Initialize the SPI and I2C ports
*/
int ArduCAM(uint8_t model)
{

	int ret;
	myCAM.sensor_model = model;

	myCAM.sensor_addr = 0x3c;
	// initialize i2c:
	if((i2c1 = open(i2cdev1, O_RDWR)) < 0)
	{
			perror("Failed to open i2c device.\n");
			exit(1);
	}

	if(ioctl(i2c1, I2C_SLAVE, myCAM.sensor_addr) < 0)
	{
			printf("Failed to access bus.\n");
			exit(1);
	}
}

/*
Function: 		InitCAM
Param: 				None
Description :	Initialize the Camera Module
*/
void InitCAM()
{
	uint8_t rtn = 0;
	uint8_t reg_val;
	wrSensorReg16_8(0x3008, 0x80);


	wrSensorRegs16_8(OV5642_RGB_QVGA);
	rdSensorReg16_8(0x3818,&reg_val);
	wrSensorReg16_8(0x3818, (reg_val | 0x60) & 0xff);
	rdSensorReg16_8(0x3621,&reg_val);
	wrSensorReg16_8(0x3621, reg_val & 0xdf);


}

uint8_t wrSensorReg8_8(uint8_t regID, uint8_t regDat)
{
	char wbuf[2]={regID,regDat}; //first byte is address to write. others are bytes to be written
	write(i2c1, wbuf, 2);
	return 1;
}

uint8_t rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{

	char read_start_buf[1] = {regID};
	char rbuf[1];
	write(i2c1, read_start_buf, 1); //reposition file pointer to register 0x28
	read(i2c1, rbuf, 1);
	*regDat = rbuf[0];
	return 1;
}

uint8_t wrSensorReg8_16(uint8_t regID, uint16_t regDat)
{
	char wbuf[3]={regID,(regDat>>8) & 0x00ff,regDat & 0x00ff}; //first byte is address to write. others are bytes to be written
	write(i2c1, wbuf, 3);
	return 1;
}

uint8_t rdSensorReg8_16(uint8_t regID, uint16_t* regDat)
{
	char read_start_buf[1] = {regID};
	char rbuf[2];
	write(i2c1, read_start_buf, 1); //reposition file pointer to register 0x28
	read(i2c1, rbuf, 2);
	*regDat =((rbuf[0]<<8) & rbuf[1]) & 0xffff;
	return 1;
}

uint8_t wrSensorReg16_8(uint16_t regID, uint8_t regDat)
{
	uint8_t reg_H,reg_L;
	uint16_t value;
	reg_H = (regID >> 8) & 0x00ff;
	reg_L = regID & 0x00ff;
	char wbuf[3]={reg_H,reg_L,regDat}; //first byte is address to write. others are bytes to be written
	write(i2c1, wbuf, 3);
	return 1;
}

uint8_t rdSensorReg16_8(uint16_t regID, uint8_t* regDat)
{
	uint8_t reg_H,reg_L;
	int r;
	reg_H = (regID >> 8) & 0x00ff;
	reg_L = regID & 0x00ff;
	char read_start_buf[2] = {reg_H,reg_L};
	char rbuf[1];
	write(i2c1, read_start_buf, 2); //reposition file pointer to register 0x28
	read(i2c1, rbuf, 1);
	*regDat =rbuf[0];
	return 1;
}

int wrSensorRegs8_8(const struct sensor_reg reglist[])
{
	int err = 0;
	uint16_t reg_addr = 0;
	uint16_t reg_val = 0;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		err = wrSensorReg8_8(reg_addr, reg_val);
   	next++;
	}
	return 1;
}


int wrSensorRegs8_16(const struct sensor_reg reglist[])
{
	int err = 0;

	unsigned int reg_addr,reg_val;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xff) | (reg_val != 0xffff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		err = wrSensorReg8_16(reg_addr, reg_val);
   	next++;
	}
	return 1;
}

int wrSensorRegs16_8(const struct sensor_reg reglist[])
{
	int err = 0;

	unsigned int reg_addr,reg_val;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xffff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		err = wrSensorReg16_8(reg_addr, reg_val);
   	next++;
	}

	return 1;
}


void delayms(int i)
{
	while(i)
	{i--;}
}
