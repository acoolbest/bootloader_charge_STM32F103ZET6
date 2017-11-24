//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
//#include "dwgl_function.h"
#ifndef __DWGL_FUNCTION_H
#define	__DWGL_FUNCTION_H

#include "stm32f10x.h"

#define wave_area_XL    0
#define wave_area_XR    320
#define wave_area_YU    50
#define wave_area_YD    180

extern void u16toStr(u16 u_data,u8 *p);	
extern void u16toStr_unit(u16 u_data,u8 *p,u8 unit);	
extern void wave_Configuration(u8 *p);
extern void wave_display(u8 type,u8 frequency,u8 *p);
extern void continuous_interpolation(u16 *p,u8 gain,u16 *out_p);
extern void continuous_display(u16 *p,u8 gain,uint16_t color);
extern void buffer_display(u16 *p,u8 gain,uint16_t color);
extern void point_display(u16 x,u16 y,uint16_t color);
extern void number_x(uint16_t x, uint16_t y,uint8_t num_x, uint16_t color);
extern void TASK0_display(void);
extern void TASK1_display(void);
extern void TASK2_display(void);
extern void Version_display(u16 x,u8 *p);

extern void Addr_Set(void);
extern void Hub_ID_Info(void);
extern void Device_Info(void);
extern void Port_Info(u8 port);
extern void Device_Rst(void);
extern void Hub_Rst(u8 addr,u8 port);	//NC
extern void Choose_uart2(u8 port);		//NC
extern void Power_on(u8 port,u16 time);
extern void Power_off(u8 port);
extern void AllPort_PowerInfo(void);
extern void File_Requst(u8 FN);
extern void File_Tx(u8 FN);
extern void File_Recall(u8 FN);
extern void File_Erase(u8 FN);
extern void RGB888_565(unsigned int addr);

extern void LCD_TEST(void);

extern u8 Frame_check_cmd1(void);
//uart1
extern void uart1_cmd (void);
extern void cmd_Erase_Flash(void);
extern void cmd_Read_Flash(void);
extern void cmd_Write_Flash(void);
extern void cmd_Get_State(void);
extern void cmd_Set_State(void);
extern void cmd_Get_ADC(void);
extern void cmd_Save_ADC(void);
extern void cmd_RGB888_565(void);
extern void cmd_RGB_clear(void);
extern void cmd_CHIP_PRO(void);
extern void cmd_PRO_Version(void);
extern void cmd_Hub_Rst(void);			//NC
extern void cmd_Device_Info(void);
extern void cmd_ShakeHands(void);
extern void cmd_Power_on(void);
extern void cmd_Power_off(void);
extern void cmd_File_Requst(void);
extern void cmd_File_Tx(void);
extern void cmd_File_Recall(void);
extern void cmd_File_Erase(void);
extern void cmd_Port_Info(void);
extern void cmd_Device_num(void);
extern void cmd_Device_Check(void);
extern void cmd_MediaCtrl(void);
extern void cmd_Set_MediaV(void);
extern void cmd_Get_MediaV(void);
extern void cmd_Set_Version(void);
extern void cmd_Get_Version(void);
#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
extern void cmd_Get_AD_count(void);
extern void cmd_Get_charge_speed(void);
#endif
//uart3
extern void uart3_cmd (void);
extern void cmd3_ShakeHands(void);
extern void cmd3_Get_State(void);
extern void cmd3_Set_State(void);

//充电
extern void Dport_ChargeState(void);
extern void Dport_ChargeState1(void);
extern void Dport_ChargeStateB(void);
extern void Dport_ChargeStateC(void);
extern void Dport_ChargeHB_ON(void);
extern void Dport_ChargeHB_OFF(void);
extern void Dport_ChargeHC_ON(void);
extern void Dport_ChargeHC_OFF(void);

extern void Dport_ChargeON(u8 port);
extern void Dport_ChargeOFF(u8 port);
extern void ChargeCtrl_B(void);
extern void ChargeCtrl_C(void);
extern void Get_ADC_BaseLine(void);
extern void DisplayADC_BL(unsigned int x, unsigned int y, u16 *s,u16 PenColor, u16 BackColor,u8 cs);
extern void DisplayPROT_EWM(unsigned int x, unsigned int y, u8 num,u8 cs);
extern void FiletoBuffer_ID(u8 area,u8 id,u8 *p);//以ID放式调读文件到BUFFER。

typedef void (*VOID_FUNC_START_ROUTINE)(void);
#define VOID_FUNC_COUNT			(11)

extern void usb_power_ctrl(u8 usb_port, u8 new_state);
extern void led_power_ctrl(u8 usb_port, u8 new_state);
extern void usb_mutually_exclusive_power_on(u8 lcd_index);
extern void rewrite_ADC_BaseLine_flash_data(void);
//extern void Dport_Charge_State(u8 lcd_index);
extern void port_Charge_State(u8 lcd_index);
extern void get_ADC1_3_data(u16 * ADC_data);

extern void process_com_data(struct cmd_recv_stru *p_cmd_recv_stru, u8 u8_recv);
extern u8 get_checksum(u8 * p_frame, u16 frame_len);


#endif
