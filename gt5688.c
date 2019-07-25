#include "bsp.h"


//GT5688���ò�����
//x����������ֵ0x0320=800
//y����������ֵ0x01E0=480
//0x43  ��һ���ֽ��ǹ̼��汾
//0x8050~0x813B  236���Ĵ���
//0x813C  0x813DУ���
//0x813E  ���ø��±��
const u8 GT5688_CFG_TBL[]=
{ 
	0x43,0x20,0x03,0xE0,0x01,0x05,0x3D,0x10,0x01,0x00,
	0x08,0x08,0x50,0x3C,0x53,0x11,0x01,0x01,0x00,0x00,
	0x14,0x14,0x14,0x22,0x08,0x04,0x00,0x00,0x00,0x00,
	0x3C,0x00,0x53,0x00,0x14,0x00,0x00,0x84,0x00,0x00,
	0x00,0x00,0x00,0x64,0x1E,0x1E,0x8A,0x2A,0x0C,0x3C,
	0x3E,0xF8,0x0A,0x20,0x33,0x60,0x12,0x02,0x24,0x00,
	0x00,0x32,0x64,0x80,0x14,0x02,0x00,0x00,0x54,0x80,
	0x35,0x7F,0x3D,0x7F,0x46,0x7F,0x51,0x7F,0x5D,0x7F,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x32,0x20,0x50,0x3C,0x3C,0x00,0x00,0x00,0x00,0x00,
	0x0D,0x06,0x0C,0x05,0x0B,0x04,0x0A,0x03,0x09,0x02,
	0x08,0x01,0xFF,0xFF,0x00,0x01,0x02,0x03,0x04,0x05,
	0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
	0x10,0x11,0x12,0x13,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0xC8,0xF5,0x01
};




//��GT5688д��һ������
//reg:��ʼ�Ĵ�����ַ
//buf:���ݻ�������
//len:д���ݳ���
//����ֵ:0,�ɹ�;1,ʧ��.
u8 GT5688_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	IIC_Start();	
 	IIC_Send_Byte(GT_CMD_WR);   	//����д���� 	 
	IIC_Wait_Ack();
	IIC_Send_Byte(reg>>8);   	//���͸�8λ��ַ
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(reg&0XFF);   	//���͵�8λ��ַ
	IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    	IIC_Send_Byte(buf[i]);  	//������
		ret=IIC_Wait_Ack();
		if(ret)break;  
	}
    IIC_Stop();					//����һ��ֹͣ����	    
	return ret; 
}
//��GT5688����һ������
//reg:��ʼ�Ĵ�����ַ
//buf:���ݻ�������
//len:�����ݳ���			  
void GT5688_RD_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i; 
 	IIC_Start();	
 	IIC_Send_Byte(GT_CMD_WR);   //����д���� 	 
	IIC_Wait_Ack();
 	IIC_Send_Byte(reg>>8);   	//���͸�8λ��ַ
	IIC_Wait_Ack(); 	 										  		   
 	IIC_Send_Byte(reg&0XFF);   	//���͵�8λ��ַ
	IIC_Wait_Ack();  
 	IIC_Start();  	 	   
	IIC_Send_Byte(GT_CMD_RD);   //���Ͷ�����		   
	IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=IIC_Read_Byte(i==(len-1)?0:1); //������	  
	} 
    IIC_Stop();//����һ��ֹͣ����    
} 


//����GT5688���ò���
//mode:0,���������浽flash
//     1,�������浽flash
u8 GT5688_Send_Cfg(u8 mode)
{

	u16 checksum=0;
	u8 buf[3];
	u8 i=0;	
	for(i=0;i<(sizeof(GT5688_CFG_TBL)-3);i+=2)
		checksum +=((GT5688_CFG_TBL[i]<<8)|GT5688_CFG_TBL[i+1]);//����У���
		//checksum +=(GT5688_CFG_TBL[i]<<8)+GT5688_CFG_TBL[i+1];
	//checksum =0-checksum;
	 checksum =(~checksum)+1;
	printf("chksum:0x%x,\r\n",checksum);
	buf[0]= checksum>>8;
	buf[1]= checksum;
	buf[2]= mode;	//�Ƿ�д�뵽GT5688 FLASH?  ���Ƿ���籣��
	GT5688_WR_Reg(GT_CFGS_REG,(u8*)GT5688_CFG_TBL,sizeof(GT5688_CFG_TBL));//���ͼĴ�������
	return 0;

} 

//��ʼ��GT5688������
//����ֵ:0,��ʼ���ɹ�;1,��ʼ��ʧ�� 
u8 GT5688_Init(void)
{
	u8 temp[5]; 
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOH_CLK_ENABLE();			//����GPIOHʱ��
	
	//PH7
	GPIO_Initure.Pin=GPIO_PIN_7;            //PH7
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
	GPIO_Initure.Pull=GPIO_PULLUP;          //����
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //��ʼ��
					
	//PI8
	GPIO_Initure.Pin=GPIO_PIN_6;            //PI8
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //��ʼ��
	
	IIC_Init();     //��ʼ����������I2C����  
	GT_RST=0;				//��λ
	delay_ms(10);
	GT_RST=1;				//�ͷŸ�λ		    
	delay_ms(10); 
	GPIO_Initure.Pin=GPIO_PIN_7;            //PH7
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
	GPIO_Initure.Pull=GPIO_NOPULL;          //��������������������
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //��ʼ��
	delay_ms(100);  
	GT5688_RD_Reg(GT_PID_REG,temp,4);//��ȡ��ƷID
	temp[4]=0;
	SEGGER_RTT_printf(0,"CTP ID:%s\r\n",temp);	//��ӡID
	if(strcmp((char*)temp,"5688")==0)//ID==5688
	{
		temp[0]=0X02;			
		GT5688_WR_Reg(GT_CTRL_REG,temp,1);//��λGT9147
		GT5688_RD_Reg(GT_CFGS_REG,temp,1);//��ȡGT_CFGS_REG�Ĵ���
		//if(temp[0]<0X60)//Ĭ�ϰ汾�Ƚϵ�,��Ҫ����flash����
		//{
		SEGGER_RTT_printf(0,"Default Ver:%d\r\n",temp[0]);
			//GT5688_Send_Cfg(1);//���²���������
		//}
		delay_ms(10);
		temp[0]=0X00;	 
		GT5688_WR_Reg(GT_CTRL_REG,temp,1);//������λ   
		return 0;
	} 
	return 1;
}

const u16 GT5688_TPX_TBL[5]={GT_TP1_REG,GT_TP2_REG,GT_TP3_REG,GT_TP4_REG,GT_TP5_REG};
//ɨ�败����(���ò�ѯ��ʽ)
//mode:0,����ɨ��.
//����ֵ:��ǰ����״̬.
//0,�����޴���;1,�����д���
u8 GT5688_Scan(u8 mode)
{
	u8 buf[4];
	u8 i=0;
	u8 res=0;
	u8 temp;
	u8 tempsta;
 	static u8 t=0;//���Ʋ�ѯ���,�Ӷ�����CPUռ����   
	t++;
	if((t%10)==0||t<10)//����ʱ,ÿ����10��CTP_Scan�����ż��1��,�Ӷ���ʡCPUʹ����
	{
		GT5688_RD_Reg(GT_GSTID_REG,&mode,1);	//��ȡ�������״̬  
 		if(mode&0X80&&((mode&0XF)<6))
		{
			temp=0;
			GT5688_WR_Reg(GT_GSTID_REG,&temp,1);//���־ 		
		}		
		if((mode&0XF)&&((mode&0XF)<6))
		{
			temp=0XFF<<(mode&0XF);		//����ĸ���ת��Ϊ1��λ��,ƥ��tp_dev.sta���� 
			tempsta=tp_dev.sta;			//���浱ǰ��tp_dev.staֵ
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
			tp_dev.x[4]=tp_dev.x[0];	//���津��0������
			tp_dev.y[4]=tp_dev.y[0];
			for(i=0;i<5;i++)
			{
				if(tp_dev.sta&(1<<i))	//������Ч?
				{
					GT5688_RD_Reg(GT5688_TPX_TBL[i],buf,4);	//��ȡXY����ֵ

                        if(!(tp_dev.touchtype&0X01))//����
                        {
                            tp_dev.y[i]=((u16)buf[1]<<8)+buf[0];
                            tp_dev.x[i]=800-(((u16)buf[3]<<8)+buf[2]);
                        }else
                        {
                            tp_dev.x[i]=((u16)buf[1]<<8)+buf[0];
                            tp_dev.y[i]=((u16)buf[3]<<8)+buf[2];
                        }  
					SEGGER_RTT_printf(0,"x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]); //Jlink��ӡ����
				}			
			} 
			res=1;
			if(tp_dev.x[0]>lcddev.width||tp_dev.y[0]>lcddev.height)//�Ƿ�����(���곬����)
			{ 
				if((mode&0XF)>1)		//��������������,�򸴵ڶ�����������ݵ���һ������.
				{
					tp_dev.x[0]=tp_dev.x[1];
					tp_dev.y[0]=tp_dev.y[1];
					t=0;				//����һ��,��������������10��,�Ӷ����������
				}else					//�Ƿ�����,����Դ˴�����(��ԭԭ����)  
				{
					tp_dev.x[0]=tp_dev.x[4];
					tp_dev.y[0]=tp_dev.y[4];
					mode=0X80;		
					tp_dev.sta=tempsta;	//�ָ�tp_dev.sta
				}
			}else t=0;					//����һ��,��������������10��,�Ӷ����������
		}
	}
	if((mode&0X8F)==0X80)//�޴����㰴��
	{ 
		if(tp_dev.sta&TP_PRES_DOWN)	//֮ǰ�Ǳ����µ�
		{
			tp_dev.sta&=~(1<<7);	//��ǰ����ɿ�
		}else						//֮ǰ��û�б�����
		{ 
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=0XE0;	//�������Ч���	
		}	 
	} 	
	if(t>240)t=10;//���´�10��ʼ����
	return res;
}
 



























