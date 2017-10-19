///////////////////////////////////////////////
//  file name:Address_RAM.h
//  writer: dwgl 2015-0601 
//  version: v1.01
//  devece:  FM25L64-GA
///////////////////////////////////////////////


#include "compiler_defs.h"
#include "C8051F340_defs.h"

#ifndef Address_RAM.h
#define Address_RAM.h


sbit  RAM_CS           =  P1^4;

#define  DATARAM_CS_En()    RAM_CS = 0
#define  DATARAM_CS_Dis()   RAM_CS = 1

void  RAM_Write_OneByte(U8 ADDRESS_H, U8 ADDRESS_L, U8 DATA);
U8    RAM_Read_OneByte(U8 ADDRESS_H, U8 ADDRESS_L);
void  RAM_Write_OneWord(U8 ADDRESS_H, U8 ADDRESS_L, U8 DATA1, U8 DATA2);
U16   RAM_Read_OneWord(U8 ADDRESS_H, U8 ADDRESS_L);
void  RAM_Write_DoubleWord(U8 ADDRESS_H, U8 ADDRESS_L, U8 DATA1, U8 DATA2, U8 DATA3, U8 DATA4);
U32   RAM_Read_DoubleWord(U8 ADDRESS_H, U8 ADDRESS_L);
void  RAM_Write_String(U8 ADDRESS_H, U8 ADDRESS_L, U8 * SPI_Pointer, U16 SPI_Length);
void  RAM_Read_String(U8 ADDRESS1, U8 ADDRESS2, U8 * SPI_Pointer, U8 SPI_Length);
void  RAM_Write_SameString(U8 ADDRESS_H, U8 ADDRESS_L, U8 DATA, U16 SPI_Length);


#endif
