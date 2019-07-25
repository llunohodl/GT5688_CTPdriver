#ifndef __TOUCH_H__
#define __TOUCH_H__
#include "bsp.h"    

#define TP_PRES_DOWN 0x80  //Touch screen is pressed	  
#define TP_CATH_PRES 0x40  //There is a button pressed
#define CT_MAX_TOUCH  5    //The number of points supported by the capacitive screen is fixed at 5 points.

//
Touch screen controller
typedef struct
{
	void (*init)(void);		//Initialize the touch screen controller
	u8 (*scan)(u8);				//Scan the touch screen: 0, screen scan / 1, physical coordinates;	  
	u16 x[CT_MAX_TOUCH]; 		//Current coordinates
	u16 y[CT_MAX_TOUCH];		//The capacitive screen has a maximum of 5 sets of coordinates, 
                                //and the resistive screen uses x[0], y[0] represents: 
                                //the coordinates of the touch screen during this scan.
								//x[4], y[4] stores the coordinates when the first press. 
	u8  sta;					//Pen status:
								//b7:Press 1 to release 0; 
	                            //b6:0, no button press; 1, there is a button press.
								//b5:Reserved
								//b4-b0:The number of points pressed by the capacitive 
                                //touch screen (0 means no press, 1 means press)
	u8 touchtype;
}_m_tp_dev;

extern _m_tp_dev tp_dev;	 	//The touch screen controller is defined in touch.c

u8 TP_Scan(u8 tp);				
void TP_Init(void);
 
#endif

















