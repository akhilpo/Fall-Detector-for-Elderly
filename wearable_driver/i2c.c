// i2c driver definition

#include "i2c.h"


void I2CSetup(unsigned long ulI2CBase, unsigned long ulI2CSpeed)
{
	volatile unsigned long temp, gpiomtpr;
	
	temp = uiSysClk;
	temp = RCGCI2C;
	RCGCI2C |= 1;
//	temp = RCGCI2C;
	temp++;
	RCGCGPIO |= 2;
	GPIOHBCTL_B |= 2;	//Enable AHB for GPIO Port B
	GPIOAFSEL_B |= (1<<2) | (1<<3);
	GPIOPCTL_B |= (0x3<<8) | (0x3<<12); //Enable I2C0 in GPIO PORT B PCTL
	//temp = GPIOPCTL_B;
	GPIOODR_B |= (1<<3);	//Set I2CSDA pin as open drain
	GPIODR2R |= (1<<2) | (1<<3);
	GPIOPUR |= (1<<3);

	I2CMCR |= (1<<4);
	I2CMTPR = (uiSysClk/(2*10*100000)) - 1;
	gpiomtpr = I2CMTPR;
	
}

unsigned long I2CRegRead(unsigned long ulI2CBase, unsigned char ucSlaveAdress, unsigned char ucReg)
{
	while (I2CMCS & BUSY);
	I2CMSA = (ucSlaveAddrress << 1);
	I2CMDR = ucReg;
	I2CMCS = (START|RUN|ACK);
	while (I2CMCS & BUSY);	//Wait till data is transferred
	// volatile unsigned long temp;
	// while (1)
	// {
	// 	temp = I2CMCS;
	// 	temp |= BUSBSY;
	// 	if (temp )
	// }
	if (I2CMCS & ERROR)
		return 1;

	I2CMSA |= 0x1;			//Enable slave read
	I2CMCS = (START|STOP|RUN);
	while (I2CMCS & BUSY);
	if (I2CMCS & ERROR)
		return 1;
	
	return I2CMDR;

}


unsigned long I2CRegWrite(unsigned long ulI2CBase, unsigned char ucSlaveAdress, unsigned char ucReg, unsigned char ucValue)
{
	while (I2CMCS & BUSY);
	I2CMSA = (ucSlaveAddrress << 1);
	I2CMDR = ucReg;
	I2CMCS = (START|RUN|ACK);
	while (I2CMCS & BUSY);	 

	// while (1)
	if (I2CMCS & ERROR)
		return 1;
	I2CMDR = ucValue;
	I2CMCS = (STOP|RUN); //Generate stop condition only
	while (I2CMCS & BUSY);
	if (I2CMCS & ERROR)
		return 1;
}

