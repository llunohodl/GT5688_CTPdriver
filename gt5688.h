#ifndef __GT5688_H
#define __GT5688_H	
#include "sys.h"	


//IO��������	 
#define GT_RST  PHout(6) //GT5688��λ����
#define GT_INT  PHin(7)  //GT5688�ж�����		
 
//I2C��д����	
#define GT_CMD_WR 		0X28     	//д����
#define GT_CMD_RD 		0X29		//������
  

//GT5688 ���ּĴ������� 
#define GT_CTRL_REG 	0X8040   	//GT5688���ƼĴ���
#define GT_CFGS_REG 	0X8050   	//GT5688������ʼ��ַ�Ĵ���
#define GT_CHECK_REG 	0X813C   	//GT5688У��ͼĴ���
#define GT_PID_REG 		0X8140   	//GT5688��ƷID�Ĵ���

#define GT_GSTID_REG 	0X814E   	//GT5688��ǰ��⵽�Ĵ������
#define GT_TP1_REG 		0X8150  	//��һ�����������ݵ�ַ
#define GT_TP2_REG 		0X8158		//�ڶ������������ݵ�ַ
#define GT_TP3_REG 		0X8160		//���������������ݵ�ַ
#define GT_TP4_REG 		0X8168		//���ĸ����������ݵ�ַ
#define GT_TP5_REG 		0X8170		//��������������ݵ�ַ  
 
u8 GT5688_Send_Cfg(u8 mode);
u8 GT5688_WR_Reg(u16 reg,u8 *buf,u8 len);
void GT5688_RD_Reg(u16 reg,u8 *buf,u8 len); 
u8 GT5688_Init(void);
u8 GT5688_Scan(u8 mode); 
#endif













