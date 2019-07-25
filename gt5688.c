#include "bsp.h"


//GT5688配置参数表
//x坐标输出最大值0x0320=800
//y坐标输出最大值0x01E0=480
//0x43  第一个字节是固件版本
//0x8050~0x813B  236个寄存器
//0x813C  0x813D校验和
//0x813E  配置更新标记
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




//向GT5688写入一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:写数据长度
//返回值:0,成功;1,失败.
u8 GT5688_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	IIC_Start();	
 	IIC_Send_Byte(GT_CMD_WR);   	//发送写命令 	 
	IIC_Wait_Ack();
	IIC_Send_Byte(reg>>8);   	//发送高8位地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    	IIC_Send_Byte(buf[i]);  	//发数据
		ret=IIC_Wait_Ack();
		if(ret)break;  
	}
    IIC_Stop();					//产生一个停止条件	    
	return ret; 
}
//从GT5688读出一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度			  
void GT5688_RD_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i; 
 	IIC_Start();	
 	IIC_Send_Byte(GT_CMD_WR);   //发送写命令 	 
	IIC_Wait_Ack();
 	IIC_Send_Byte(reg>>8);   	//发送高8位地址
	IIC_Wait_Ack(); 	 										  		   
 	IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	IIC_Wait_Ack();  
 	IIC_Start();  	 	   
	IIC_Send_Byte(GT_CMD_RD);   //发送读命令		   
	IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=IIC_Read_Byte(i==(len-1)?0:1); //发数据	  
	} 
    IIC_Stop();//产生一个停止条件    
} 


//发送GT5688配置参数
//mode:0,参数不保存到flash
//     1,参数保存到flash
u8 GT5688_Send_Cfg(u8 mode)
{

	u16 checksum=0;
	u8 buf[3];
	u8 i=0;	
	for(i=0;i<(sizeof(GT5688_CFG_TBL)-3);i+=2)
		checksum +=((GT5688_CFG_TBL[i]<<8)|GT5688_CFG_TBL[i+1]);//计算校验和
		//checksum +=(GT5688_CFG_TBL[i]<<8)+GT5688_CFG_TBL[i+1];
	//checksum =0-checksum;
	 checksum =(~checksum)+1;
	printf("chksum:0x%x,\r\n",checksum);
	buf[0]= checksum>>8;
	buf[1]= checksum;
	buf[2]= mode;	//是否写入到GT5688 FLASH?  即是否掉电保存
	GT5688_WR_Reg(GT_CFGS_REG,(u8*)GT5688_CFG_TBL,sizeof(GT5688_CFG_TBL));//发送寄存器配置
	return 0;

} 

//初始化GT5688触摸屏
//返回值:0,初始化成功;1,初始化失败 
u8 GT5688_Init(void)
{
	u8 temp[5]; 
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOH_CLK_ENABLE();			//开启GPIOH时钟
	
	//PH7
	GPIO_Initure.Pin=GPIO_PIN_7;            //PH7
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
	GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //初始化
					
	//PI8
	GPIO_Initure.Pin=GPIO_PIN_6;            //PI8
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //初始化
	
	IIC_Init();     //初始化电容屏的I2C总线  
	GT_RST=0;				//复位
	delay_ms(10);
	GT_RST=1;				//释放复位		    
	delay_ms(10); 
	GPIO_Initure.Pin=GPIO_PIN_7;            //PH7
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
	GPIO_Initure.Pull=GPIO_NOPULL;          //不带上下拉，浮空输入
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //初始化
	delay_ms(100);  
	GT5688_RD_Reg(GT_PID_REG,temp,4);//读取产品ID
	temp[4]=0;
	SEGGER_RTT_printf(0,"CTP ID:%s\r\n",temp);	//打印ID
	if(strcmp((char*)temp,"5688")==0)//ID==5688
	{
		temp[0]=0X02;			
		GT5688_WR_Reg(GT_CTRL_REG,temp,1);//软复位GT9147
		GT5688_RD_Reg(GT_CFGS_REG,temp,1);//读取GT_CFGS_REG寄存器
		//if(temp[0]<0X60)//默认版本比较低,需要更新flash配置
		//{
		SEGGER_RTT_printf(0,"Default Ver:%d\r\n",temp[0]);
			//GT5688_Send_Cfg(1);//更新并保存配置
		//}
		delay_ms(10);
		temp[0]=0X00;	 
		GT5688_WR_Reg(GT_CTRL_REG,temp,1);//结束复位   
		return 0;
	} 
	return 1;
}

const u16 GT5688_TPX_TBL[5]={GT_TP1_REG,GT_TP2_REG,GT_TP3_REG,GT_TP4_REG,GT_TP5_REG};
//扫描触摸屏(采用查询方式)
//mode:0,正常扫描.
//返回值:当前触屏状态.
//0,触屏无触摸;1,触屏有触摸
u8 GT5688_Scan(u8 mode)
{
	u8 buf[4];
	u8 i=0;
	u8 res=0;
	u8 temp;
	u8 tempsta;
 	static u8 t=0;//控制查询间隔,从而降低CPU占用率   
	t++;
	if((t%10)==0||t<10)//空闲时,每进入10次CTP_Scan函数才检测1次,从而节省CPU使用率
	{
		GT5688_RD_Reg(GT_GSTID_REG,&mode,1);	//读取触摸点的状态  
 		if(mode&0X80&&((mode&0XF)<6))
		{
			temp=0;
			GT5688_WR_Reg(GT_GSTID_REG,&temp,1);//清标志 		
		}		
		if((mode&0XF)&&((mode&0XF)<6))
		{
			temp=0XFF<<(mode&0XF);		//将点的个数转换为1的位数,匹配tp_dev.sta定义 
			tempsta=tp_dev.sta;			//保存当前的tp_dev.sta值
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
			tp_dev.x[4]=tp_dev.x[0];	//保存触点0的数据
			tp_dev.y[4]=tp_dev.y[0];
			for(i=0;i<5;i++)
			{
				if(tp_dev.sta&(1<<i))	//触摸有效?
				{
					GT5688_RD_Reg(GT5688_TPX_TBL[i],buf,4);	//读取XY坐标值

                        if(!(tp_dev.touchtype&0X01))//横屏
                        {
                            tp_dev.y[i]=((u16)buf[1]<<8)+buf[0];
                            tp_dev.x[i]=800-(((u16)buf[3]<<8)+buf[2]);
                        }else
                        {
                            tp_dev.x[i]=((u16)buf[1]<<8)+buf[0];
                            tp_dev.y[i]=((u16)buf[3]<<8)+buf[2];
                        }  
					SEGGER_RTT_printf(0,"x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]); //Jlink打印数据
				}			
			} 
			res=1;
			if(tp_dev.x[0]>lcddev.width||tp_dev.y[0]>lcddev.height)//非法数据(坐标超出了)
			{ 
				if((mode&0XF)>1)		//有其他点有数据,则复第二个触点的数据到第一个触点.
				{
					tp_dev.x[0]=tp_dev.x[1];
					tp_dev.y[0]=tp_dev.y[1];
					t=0;				//触发一次,则会最少连续监测10次,从而提高命中率
				}else					//非法数据,则忽略此次数据(还原原来的)  
				{
					tp_dev.x[0]=tp_dev.x[4];
					tp_dev.y[0]=tp_dev.y[4];
					mode=0X80;		
					tp_dev.sta=tempsta;	//恢复tp_dev.sta
				}
			}else t=0;					//触发一次,则会最少连续监测10次,从而提高命中率
		}
	}
	if((mode&0X8F)==0X80)//无触摸点按下
	{ 
		if(tp_dev.sta&TP_PRES_DOWN)	//之前是被按下的
		{
			tp_dev.sta&=~(1<<7);	//标记按键松开
		}else						//之前就没有被按下
		{ 
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=0XE0;	//清除点有效标记	
		}	 
	} 	
	if(t>240)t=10;//重新从10开始计数
	return res;
}
 



























