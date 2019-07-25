#ifndef __GT5688_H
#define __GT5688_H	
#include "sys.h"	


//IO Operation function
#define GT_RST  PHout(6) //GT5688 Reset pin
#define GT_INT  PHin(7)  //GT5688 Interrupt pin		
 
//Read and write I2C addreses
#define GT_CMD_WR 		0X28
#define GT_CMD_RD 		0X29
  

//GT5688  Partial register definition 
#define GT_CTRL_REG 	0X8040   	//GT5688 Control register
#define GT_CFGS_REG 	0X8050   	//GT5688 Configuration start address register
#define GT_CHECK_REG 	0X813C   	//GT5688 Checksum register
#define GT_PID_REG 		0X8140   	//GT5688 Product ID register

#define GT_GSTID_REG 	0X814E   	//GT5688  Currently detected touch situation
#define GT_TP1_REG 		0X8150  	//First touch point data address
#define GT_TP2_REG 		0X8158		//Second touch point data address
#define GT_TP3_REG 		0X8160		//Third touch point data address
#define GT_TP4_REG 		0X8168		//Fourth touch point data address
#define GT_TP5_REG 		0X8170		//Fifth touch point data address  
 
u8 GT5688_Send_Cfg(u8 mode);
u8 GT5688_WR_Reg(u16 reg,u8 *buf,u8 len);
void GT5688_RD_Reg(u16 reg,u8 *buf,u8 len); 
u8 GT5688_Init(void);
u8 GT5688_Scan(u8 mode); 
#endif













