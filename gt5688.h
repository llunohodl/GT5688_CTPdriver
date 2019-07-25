#ifndef __GT5688_H
#define __GT5688_H	
#include "sys.h"	


//IO操作函数	 
#define GT_RST  PHout(6) //GT5688复位引脚
#define GT_INT  PHin(7)  //GT5688中断引脚		
 
//I2C读写命令	
#define GT_CMD_WR 		0X28     	//写命令
#define GT_CMD_RD 		0X29		//读命令
  

//GT5688 部分寄存器定义 
#define GT_CTRL_REG 	0X8040   	//GT5688控制寄存器
#define GT_CFGS_REG 	0X8050   	//GT5688配置起始地址寄存器
#define GT_CHECK_REG 	0X813C   	//GT5688校验和寄存器
#define GT_PID_REG 		0X8140   	//GT5688产品ID寄存器

#define GT_GSTID_REG 	0X814E   	//GT5688当前检测到的触摸情况
#define GT_TP1_REG 		0X8150  	//第一个触摸点数据地址
#define GT_TP2_REG 		0X8158		//第二个触摸点数据地址
#define GT_TP3_REG 		0X8160		//第三个触摸点数据地址
#define GT_TP4_REG 		0X8168		//第四个触摸点数据地址
#define GT_TP5_REG 		0X8170		//第五个触摸点数据地址  
 
u8 GT5688_Send_Cfg(u8 mode);
u8 GT5688_WR_Reg(u16 reg,u8 *buf,u8 len);
void GT5688_RD_Reg(u16 reg,u8 *buf,u8 len); 
u8 GT5688_Init(void);
u8 GT5688_Scan(u8 mode); 
#endif













