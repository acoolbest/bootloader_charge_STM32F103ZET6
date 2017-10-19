//-------------------------------------------------------------------------------
//Address_RAM.c
//-------------------------------------------------------------------------------
//64K bit Ferroelectric Nonvolatile RAM  
//  -Organized as 8,192 x 8 bits 
//  -Unlimited Read/Write Cycles 
//  -10 Year Data Retention  
//  -NoDelay Writes 
//  -Advanced High-Reliability Ferroelectric Process 
 
//Very Fast Serial Peripheral Interface - SPI 
//  -Up to 20 MHz Frequency  
//  -Direct Hardware Replacement for EEPROM 
//  -SPI Mode 0 & 3 (CPOL, CPHA=0,0 & 1,1) 
 
 
//Sophisticated Write Protection Scheme 
//  -Hardware Protection 
//  -Software Protection 
 
//Low Power Consumption 
//  -Low Voltage Operation 2.7-3.65V 
//  -1  ¦ÌA Standby Current 
 
//Industry Standard Configuration 
//  -Industrial Temperature -40 ¡ãC to +85¡ãC 
//  -8-pin SOIC

//-------------------------------------------------------------------------------
//Includes
//-------------------------------------------------------------------------------
#include "compiler_defs.h"
#include "C8051F340_defs.h"
#include "address_RAM.h"

//-------------------------------------------------------------------------------
//Constants
//-------------------------------------------------------------------------------
#define  RAM_CMD_WREN      0x06			//WREN  Set Write Enable Latch  0000 0110b
#define  RAM_CMD_WRDI      0x04			//WRDI  Write Disable           0000 0100b
#define  RAM_CMD_RDSR      0x05			//RDSR  Read Status Register    0000 0101b
#define  RAM_CMD_WRSR      0x01			//WRSR  Write Status Register   0000 0001b
#define  RAM_CMD_READ      0x03			//READ  Read Memory Data        0000 0011b 
#define  RAM_CMD_WRITE     0X02			//WRITE  Write Memory Data      0000 0010b

//-------------------------------------------------------------------------------
//Subroutines
//-------------------------------------------------------------------------------
void RAM_Write_OneWord(U8 ADDRESS_H, U8 ADDRESS_L, U8 DATA1, U8 DATA2)  
{
  SPIEN  = 1;

  DATARAM_CS_En();
  SPI0DAT = RAM_CMD_WREN;
  while(!SPIF);
  SPIF = 0;
  DATARAM_CS_Dis();

  DATARAM_CS_En();
  SPI0DAT = RAM_CMD_WRITE;
  while(!SPIF);
  SPIF = 0;
  
  SPI0DAT = ADDRESS_H;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = ADDRESS_L;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = DATA1;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = DATA2;
  while(!SPIF);
  SPIF = 0;

  DATARAM_CS_Dis();
  SPIEN = 0;  
}


U16 RAM_Read_OneWord(U8 ADDRESS_H, U8 ADDRESS_L)
{
  UU16 word;  

  SPIEN = 1;
  DATARAM_CS_En();

  SPI0DAT = RAM_CMD_READ;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = ADDRESS_H;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = ADDRESS_L;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = 0;
  while(!SPIF);
  SPIF = 0;
  word.U8[0] = SPI0DAT;

  SPI0DAT = 0;
  while(!SPIF);
  SPIF = 0;
  word.U8[1] = SPI0DAT;

  DATARAM_CS_Dis();
  SPIEN = 0;

  return (word.U16);
}


void RAM_Write_DoubleWord(U8 ADDRESS_H, U8 ADDRESS_L, U8 DATA1, U8 DATA2, U8 DATA3, U8 DATA4)  
{
  SPIEN  = 1;

  DATARAM_CS_En();
  SPI0DAT = RAM_CMD_WREN;
  while(!SPIF);
  SPIF = 0;
  DATARAM_CS_Dis();

  DATARAM_CS_En();
  SPI0DAT = RAM_CMD_WRITE;
  while(!SPIF);
  SPIF = 0;
  
  SPI0DAT = ADDRESS_H;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = ADDRESS_L;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = DATA1;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = DATA2;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = DATA3;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = DATA4;
  while(!SPIF);
  SPIF = 0;

  DATARAM_CS_Dis();
  SPIEN = 0;  
}


U32 RAM_Read_DoubleWord(U8 ADDRESS_H, U8 ADDRESS_L)		
{
  UU32 doubleword;  

  SPIEN = 1;
  DATARAM_CS_En();

  SPI0DAT = RAM_CMD_READ;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = ADDRESS_H;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = ADDRESS_L;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = 0;
  while(!SPIF);
  SPIF = 0;
  doubleword.U8[0] = SPI0DAT;

  SPI0DAT = 0;
  while(!SPIF);
  SPIF = 0;
  doubleword.U8[1] = SPI0DAT;

  SPI0DAT = 0;
  while(!SPIF);
  SPIF = 0;
  doubleword.U8[2] = SPI0DAT;

  SPI0DAT = 0;
  while(!SPIF);
  SPIF = 0;
  doubleword.U8[3] = SPI0DAT;

  DATARAM_CS_Dis();
  SPIEN = 0;

  return (doubleword.U32);
}


void RAM_Write_String(U8 ADDRESS_H, U8 ADDRESS_L, U8 * SPI_Pointer, U16 SPI_Length)
{
  if(SPI_Length > 8192) return;
  
  SPIEN = 1;

  DATARAM_CS_En();
  SPI0DAT = RAM_CMD_WREN;
  while(!SPIF);
  SPIF = 0;
  DATARAM_CS_Dis();

  DATARAM_CS_En();
  SPI0DAT = RAM_CMD_WRITE;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = ADDRESS_H;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = ADDRESS_L;
  while(!SPIF);
  SPIF = 0;

  while(SPI_Length)
   {
     SPI0DAT = *(SPI_Pointer);
	 while(!SPIF);
     SPIF = 0;

	 SPI_Pointer++;
	 SPI_Length--;
   }

   DATARAM_CS_Dis();
   SPIEN = 0;
  
}


void RAM_Read_String(U8 ADDRESS1, U8 ADDRESS2, U8 * SPI_Pointer, U8 SPI_Length)
{
  if(SPI_Length > 8192) return;

  SPIEN =1;
  DATARAM_CS_En();

  SPI0DAT = RAM_CMD_READ;
  while(!SPIF);
  SPIF = 0;
  
  SPI0DAT = ADDRESS1;
  while(!SPIF);
  SPIF = 0;
  
  SPI0DAT = ADDRESS2;
  while(!SPIF);
  SPIF = 0;
  
  while(SPI_Length)
   {
     SPI0DAT = 0;
	 while(!SPIF);
     SPIF = 0;
	 *(SPI_Pointer) = SPI0DAT;

	 SPI_Pointer++;
	 SPI_Length--;
   }
   
  DATARAM_CS_Dis();
  SPIEN = 0;  

}


void RAM_Write_SameString(U8 ADDRESS_H, U8 ADDRESS_L, U8 DATA, U16 SPI_Length)
{
  if(SPI_Length > 8192) return;
  
  SPIEN = 1;

  DATARAM_CS_En();
  SPI0DAT = RAM_CMD_WREN;
  while(!SPIF);
  SPIF = 0;
  DATARAM_CS_Dis();

  DATARAM_CS_En();
  SPI0DAT = RAM_CMD_WRITE;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = ADDRESS_H;
  while(!SPIF);
  SPIF = 0;

  SPI0DAT = ADDRESS_L;
  while(!SPIF);
  SPIF = 0;

  while(SPI_Length)
   {
     SPI0DAT = DATA;
	 while(!SPIF);
     SPIF = 0;
	 SPI_Length--;
   }

   DATARAM_CS_Dis();
   SPIEN = 0; 
}