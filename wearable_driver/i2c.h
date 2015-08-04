// I2C driver for Stellaric MCU

#define I2CBASEADDR_0			0x40020000 //I2C0 Base Address

//I2C Registers

#define I2CMSA				*((volatile unsigned long *)(I2CBASEADDR_0 + 0x000))
#define I2CMCS				*((volatile unsigned long *)(I2CBASEADDR_0 + 0x004))
#define I2CMDR				*((volatile unsigned long *)(I2CBASEADDR_0 + 0x008))
#define I2CMTPR				*((volatile unsigned long *)(I2CBASEADDR_0 + 0x00C))
#define I2CMIMR				*((volatile unsigned long *)(I2CBASEADDR_0 + 0x010))
#define I2CMRIS				*((volatile unsigned long *)(I2CBASEADDR_0 + 0x014))
#define I2CMMIS				*((volatile unsigned long *)(I2CBASEADDR_0 + 0x018))
#define I2CMICR				*((volatile unsigned long *)(I2CBASEADDR_0 + 0x01C))
#define I2CMCR				*((volatile unsigned long *)(I2CBASEADDR_0 + 0x020))
#define I2CMCLKOCNT			*((volatile unsigned long *)(I2CBASEADDR_0 + 0x024))
#define I2CMBMON			*((volatile unsigned long *)(I2CBASEADDR_0 + 0x02C))
#define I2CMCR2				*((volatile unsigned long *)(I2CBASEADDR_0 + 0x038))
#define I2CPP				*((volatile unsigned long *)(I2CBASEADDR_0 + 0xFC0))
#define I2CPC				*((volatile unsigned long *)(I2CBASEADDR_0 + 0xFC4))

//Other Registers

#define SYSCONTROLBASEADDR	0x400FE000
#define GPIOBASEADDR_B		0x40059000				

#define DID0				*((volatile unsigned long *)(SYSCONTROLBASEADDR + 0x000))
#define RCGCI2C				*((volatile unsigned long *)(SYSCONTROLBASEADDR + 0x620))
#define RCGCGPIO			*((volatile unsigned long *)(SYSCONTROLBASEADDR + 0x608))
#define GPIOHBCTL_B			*((volatile unsigned long *)(SYSCONTROLBASEADDR + 0x06C))
#define GPIOAFSEL_B			*((volatile unsigned long *)(GPIOBASEADDR_B + 0x420))
#define GPIOPCTL_B			*((volatile unsigned long *)(GPIOBASEADDR_B + 0x52C))
#define GPIOODR_B			*((volatile unsigned long *)(GPIOBASEADDR_B + 0x50C))
#define GPIODR2R			*((volatile unsigned long *)(GPIOBASEADDR_B + 0x500))
#define GPIOPUR				*((volatile unsigned long *)(GPIOBASEADDR_B + 0x510))

//I2C CMCS BITS
#define RUN					(1<<0)
#define START				(1<<1)
#define STOP				(1<<2)
#define ACK					(1<<3)
#define HS					(1<<4)
#define BUSY				(1<<0)
#define ERROR				(1<<1)
#define ADRACK				(1<<2)
#define DATACK				(1<<3)
#define ARBLST				(1<<4)
#define IDLE				(1<<5)
#define BUSBSY				(1<<6)

/*
//Function Prototype
void i2c0Init(unsigned long uiSysClk);
unsigned char i2c0ReadReg(unsigned char ucSlaveAddr, unsigned char ulRegAddr, unsigned char *ucData);
unsigned char i2c0WriteReg(unsigned char ucSlaveAddr, unsigned char ulRegAddr, unsigned char *ucData);
*/

extern void I2CSetup(unsigned long ulI2CPeriph, unsigned long ulI2CSpeed);
extern unsigned long I2CRegRead(unsigned long ulI2CBase, unsigned char ucSlaveAdress, unsigned char ucReg);
extern unsigned long I2CRegWrite(unsigned long ulI2CBase, unsigned char ucSlaveAdress, unsigned char ucReg, unsigned char ucValue);
