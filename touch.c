#include "touch.h" 
	  
/*
  Touch the button to scan
  tp:0, screen coordinates; 1, physical coordinates (for special occasions such as calibration)
  Return value: the current touch screen state.
  0, touch screen has no touch; 1, touch screen has touch
*/
u8 TP_Scan(u8 tp)
{			   
	if(PEN==0)//Have button press
	{
		if(tp)TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]);//Read physical coordinates
		else if(TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]))//Read screen coordinates
		{
	 		tp_dev.x[0]=tp_dev.xfac*tp_dev.x[0]+tp_dev.xoff;//Convert results to screen coordinates
			tp_dev.y[0]=tp_dev.yfac*tp_dev.y[0]+tp_dev.yoff;  
	 	} 
		if((tp_dev.sta&TP_PRES_DOWN)==0)//Not pressed before
		{		 
			tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//Button press 
			tp_dev.x[4]=tp_dev.x[0];//Record the coordinates when the first press
			tp_dev.y[4]=tp_dev.y[0];  	   			 
		}			   
	}else
	{
		if(tp_dev.sta&TP_PRES_DOWN)//Previously pressed
		{
			tp_dev.sta&=~(1<<7);//Mark button release
		}else//Not pressed before
		{
			tp_dev.x[4]=0;
			tp_dev.y[4]=0;
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
		}	    
	}
	return tp_dev.sta&TP_PRES_DOWN;//Returns the current touch screen status
}  


/*
  touch screen initialization
  Return value: 0, no calibration
  1, calibrated
*/

void TP_Init(void){	
	GT5688_Init();
	tp_dev.scan=GT5688_Scan;		
	tp_dev.touchtype|=0X80;			    //Capacitive screen 
	tp_dev.touchtype|=lcddev.dir&0X01;  //Horizontal screen or vertical screen
}


