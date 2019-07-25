#include "bsp.h"

/*
  GT5688 configuration parameter table
  x coordinate output maximum value 0x0320=800
  y coordinate output maximum value 0x01E0=480
  0x43 The first byte is the firmware version
  0x8050~0x813B 236 registers
  0x813C 0x813D checksum
  0x813E configuration update tag
*/
const u8 GT5688_CFG_TBL[]=
{ 
	0x43,
    0x20,0x03,//x
    0xE0,0x01,//y
    0x05,0x3D,0x10,0x01,0x00,
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



/*
  Write data to GT5688 once
  reg: starting register address
  buf: data buffer area
  len: write data length
  Return value: 0, success; 1, failure.
*/
u8 GT5688_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	IIC_Start();	
 	IIC_Send_Byte(GT_CMD_WR);   //Send a write command	 
	IIC_Wait_Ack();
	IIC_Send_Byte(reg>>8);   	//Send high 8-bit address
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(reg&0XFF);   	//Send low 8-bit address
	IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    	IIC_Send_Byte(buf[i]);  	//Send data
		ret=IIC_Wait_Ack();
		if(ret)break;  
	}
    IIC_Stop();					//Generate a stop condition	    
	return ret; 
}
/*
  Read data from GT5688 once
  reg: starting register address
  buf: data buffer area
  len: read data length
*/	  
void GT5688_RD_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i; 
 	IIC_Start();	
 	IIC_Send_Byte(GT_CMD_WR);   //Send a write command 	 
	IIC_Wait_Ack();
 	IIC_Send_Byte(reg>>8);   	//Send high 8-bit address
	IIC_Wait_Ack(); 	 										  		   
 	IIC_Send_Byte(reg&0XFF);   	//Send low 8-bit address
	IIC_Wait_Ack();  
 	IIC_Start();  	 	   
	IIC_Send_Byte(GT_CMD_RD);   //Send read command 
	IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=IIC_Read_Byte(i==(len-1)?0:1); //Recive data	  
	} 
    IIC_Stop();//Generate a stop condition    
} 



/* 
  Send GT5688 configuration parameters
  mode: 0, the parameters are not saved to flash
  1, save the parameters to flash
*/
u8 GT5688_Send_Cfg(u8 mode)
{

	u16 checksum=0;
	u8 buf[3];
	u8 i=0;	
	for(i=0;i<(sizeof(GT5688_CFG_TBL)-3);i+=2)
		checksum +=((GT5688_CFG_TBL[i]<<8)|GT5688_CFG_TBL[i+1]);//Calculate checksum
		//checksum +=(GT5688_CFG_TBL[i]<<8)+GT5688_CFG_TBL[i+1];
	//checksum =0-checksum;
	 checksum =(~checksum)+1;
	printf("chksum:0x%x,\r\n",checksum);
	buf[0]= checksum>>8;
	buf[1]= checksum;
	buf[2]= mode;	//Is it written to GT5688 FLASH?
	GT5688_WR_Reg(GT_CFGS_REG,(u8*)GT5688_CFG_TBL,sizeof(GT5688_CFG_TBL));//Transmit register configuration
	return 0;

} 


/*
  Initialize the GT5688 touch screen
  Return value: 0, initialization is successful; 1, initialization failed
*/
u8 GT5688_Init(void)
{
	u8 temp[5]; 
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOH_CLK_ENABLE();			//Turn on the GPIOH clock
	
	GPIO_Initure.Pin=GPIO_PIN_7;            
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      
	GPIO_Initure.Pull=GPIO_PULLUP;          
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);
					
	GPIO_Initure.Pin=GPIO_PIN_6;            
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);
	
	IIC_Init();
	GT_RST=0;
	delay_ms(10);
	GT_RST=1;
	delay_ms(10); 
	GPIO_Initure.Pin=GPIO_PIN_7;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;
	GPIO_Initure.Pull=GPIO_NOPULL;
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);
	delay_ms(100);  
    
	GT5688_RD_Reg(GT_PID_REG,temp,4);//Read product ID
	temp[4]=0;
	if(strcmp((char*)temp,"5688")==0)//ID==5688
	{
		temp[0]=0X02;			
		GT5688_WR_Reg(GT_CTRL_REG,temp,1);//Soft reset GT9147
		GT5688_RD_Reg(GT_CFGS_REG,temp,1);//Read the GT_CFGS_REG register
//if(temp[0]<0X60)//The default version is lower, you need to update the flash configuration.
		//{
		SEGGER_RTT_printf(0,"Default Ver:%d\r\n",temp[0]);
			//GT5688_Send_Cfg(1);//Update and save the configuration
		//}
		delay_ms(10);
		temp[0]=0X00;	 
		GT5688_WR_Reg(GT_CTRL_REG,temp,1);//End reset  
		return 0;
	} 
	return 1;
}

const u16 GT5688_TPX_TBL[5]={GT_TP1_REG,GT_TP2_REG,GT_TP3_REG,GT_TP4_REG,GT_TP5_REG};


/* 
   Scan the touch screen (using the query method)
   mode: 0, normal scan.
   Return value: the current touch screen state.
   0, touch screen has no touch; 1, touch screen has touch
*/

u8 GT5688_Scan(u8 mode)
{
	u8 buf[4];
	u8 i=0;
	u8 res=0;
	u8 temp;
	u8 tempsta;
 	static u8 t=0;//Control the query interval to reduce CPU usage   
	t++;
	if((t%10)==0||t<10)//When idle, it will be detected once every 10 times of CTP_Scan function, thus saving CPU usage.
	{
		GT5688_RD_Reg(GT_GSTID_REG,&mode,1);	//Read the status of the touch point  
 		if(mode&0X80&&((mode&0XF)<6))
		{
			temp=0;
			GT5688_WR_Reg(GT_GSTID_REG,&temp,1);//Clear flag 		
		}		
		if((mode&0XF)&&((mode&0XF)<6))
		{
			temp=0XFF<<(mode&0XF);		//Convert the number of points to 1 digit, matching the tp_dev.sta definition
			tempsta=tp_dev.sta;			//Save the current tp_dev.sta value
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
			tp_dev.x[4]=tp_dev.x[0];	//Save contact 0 data
			tp_dev.y[4]=tp_dev.y[0];
			for(i=0;i<5;i++)
			{
				if(tp_dev.sta&(1<<i))	//Touch effective?
				{
					GT5688_RD_Reg(GT5688_TPX_TBL[i],buf,4);	//Read XY coordinate values

                        if(!(tp_dev.touchtype&0X01))//Horizontal screen
                        {
                            tp_dev.y[i]=((u16)buf[1]<<8)+buf[0];
                            tp_dev.x[i]=800-(((u16)buf[3]<<8)+buf[2]);
                        }else
                        {
                            tp_dev.x[i]=((u16)buf[1]<<8)+buf[0];
                            tp_dev.y[i]=((u16)buf[3]<<8)+buf[2];
                        }  
				}			
			} 
			res=1;
			if(tp_dev.x[0]>lcddev.width||tp_dev.y[0]>lcddev.height)//Illegal data (coordinates are out)
			{ 
				if((mode&0XF)>1)		//If there are other points with data, the data of the second contact is copied to the first contact.
				{
					tp_dev.x[0]=tp_dev.x[1];
					tp_dev.y[0]=tp_dev.y[1];
					t=0;				//Trigger once, it will monitor at least 10 times in a row, thus improving the hit rate.
				}else					//Illegal data, ignore this data (restore the original) 
				{
					tp_dev.x[0]=tp_dev.x[4];
					tp_dev.y[0]=tp_dev.y[4];
					mode=0X80;		
					tp_dev.sta=tempsta;	//Restore tp_dev.sta
				}
			}else t=0;					//Trigger once, it will monitor at least 10 times in a row, thus improving the hit rate.
		}
	}
	if((mode&0X8F)==0X80)//No touch point press
	{ 
		if(tp_dev.sta&TP_PRES_DOWN)	//Previously pressed
		{
			tp_dev.sta&=~(1<<7);	//Mark button release
		}else						//Not pressed before
		{ 
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=0XE0;	//Clear point valid flag
		}	 
	} 	
	if(t>240)t=10;//Recount from 10
	return res;
}
