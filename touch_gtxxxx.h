#ifndef _TOUCH_GTXXXX_H
#define _TOUCH_GTXXXX_H

#include "stm32f4xx_hal.h"

#define LCD_PIXEL_WIDTH  800
#define LCD_PIXEL_HEIGHT 480

struct i2c_msg {
	uint8_t addr;		/* I2C device address of the slave device */
	uint16_t flags;	    /* control mark */
	uint16_t len;		/* Length of read and write data */
	uint8_t *buf;		/* pointer to store read and write data */
};

typedef struct{
  /*Configure according to touch screen type*/
  uint16_t max_width;  //Contact maximum, high
  uint16_t max_height; //contact maximum, width
  uint16_t config_reg_addr;  	//configuration register address is different
}TOUCH_PARAM_TypeDef;

typedef enum {
	GT9157=0,
	GT911=1,
    GT5688=2,
}TOUCH_IC;

extern TOUCH_IC touchIC;
extern const TOUCH_PARAM_TypeDef touch_param[];

#define CTP_MAX_HEIGHT   touch_param[touchIC].max_height
#define CTP_MAX_WIDTH    touch_param[touchIC].max_width
#define CTP_INT_TRIGGER             0
#define CTP_MAX_TOUCH               5

#define CTP_POLL_TIME               10    
#define CTP_ADDR_LENGTH             2
#define CTP_CONFIG_MIN_LENGTH       186
#define CTP_CONFIG_MAX_LENGTH       256
#define FAIL                        0
#define SUCCESS                     1
#define SWITCH_OFF                  0
#define SWITCH_ON                   1

#define CTP_REG_BAK_REF             0x99D0
#define CTP_REG_MAIN_CLK            0x8020
#define CTP_REG_CHIP_TYPE           0x8000
#define CTP_REG_HAVE_KEY            0x804E
#define CTP_REG_MATRIX_DRVNUM       0x8069     
#define CTP_REG_MATRIX_SENNUM       0x806A
#define CTP_REG_COMMAND				0x8040

#define CTP_COMMAND_READSTATUS	    0
#define CTP_COMMAND_DIFFERENCE	    1
#define CTP_COMMAND_SOFTRESET		2
#define CTP_COMMAND_UPDATE	        3
#define CTP_COMMAND_CALCULATE	    4
#define CTP_COMMAND_TURNOFF	    	5

#define CTP_FL_FW_BURN              0x00
#define CTP_FL_ESD_RECOVERY         0x01
#define CTP_FL_READ_REPAIR          0x02

#define CTP_BAK_REF_SEND                0
#define CTP_BAK_REF_STORE               1
#define CFG_LOC_DRVA_NUM                29
#define CFG_LOC_DRVB_NUM                30
#define CFG_LOC_SENS_NUM                31

#define CTP_CHK_FW_MAX                  40
#define CTP_CHK_FS_MNT_MAX              300
#define CTP_BAK_REF_PATH                "/data/gtp_ref.bin"
#define CTP_MAIN_CLK_PATH               "/data/gtp_clk.bin"
#define CTP_RQST_CONFIG                 0x01
#define CTP_RQST_BAK_REF                0x02
#define CTP_RQST_RESET                  0x03
#define CTP_RQST_MAIN_CLOCK             0x04
#define CTP_RQST_RESPONDED              0x00
#define CTP_RQST_IDLE                   0xFF

// Registers define
#define CTP_READ_COOR_ADDR    0x814E
#define CTP_REG_SLEEP         0x8040
#define CTP_REG_SENSOR_ID     0x814A
#define CTP_REG_CONFIG_DATA   touch_param[touchIC].config_reg_addr
#define CTP_REG_VERSION       0x8140

#define RESOLUTION_LOC        3
#define TRIGGER_LOC           8
#define X2Y_LOC        		 (1<<3)

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))
	                         
#define	TS_ACT_NONE	 	0
#define	TS_ACT_DOWN	 	1
#define   TS_ACT_UP		2
			

extern int16_t pre_x;
extern int16_t pre_y;
																			 
int8_t CTP_Reset_Guitar(void);
int32_t CTP_Read_Version(void);
void CTP_IRQ_Disable(void);
void CTP_IRQ_Enable(void);
int32_t CTP_Init_Panel(void);
int8_t CTP_Send_Command(uint8_t command);
void GT9xx_GetOnePiont(void);

void CTP_touch_polling();


#define CTP_DEBUG_ON         	1
#define CTP_DEBUG_ARRAY_ON      0
#define CTP_DEBUG_FUNC_ON   	0
// Log define
#define CTP_INFO(fmt,arg...)           printf("I> "fmt"\n",##arg)
#define CTP_ERROR(fmt,arg...)          printf("E> "fmt"\n",##arg)
#define CTP_DEBUG(fmt,arg...)          do{\
                                         if(CTP_DEBUG_ON)\
                                         printf("D> [%d]"fmt"\n",__LINE__, ##arg);\
																					}while(0)

#define CTP_DEBUG_ARRAY(array, num)    do{\
                                         int32_t i;\
                                         uint8_t* a = array;\
                                         if(CTP_DEBUG_ARRAY_ON)\
                                         {\
                                            printf("A>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printf("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printf("\n");\
                                                }\
                                            }\
                                            printf("\n");\
                                        }\
                                       }while(0)

#define CTP_DEBUG_FUNC()               do{\
                                         if(CTP_DEBUG_FUNC_ON)\
                                         printf("F> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)

																			 
																			 
#define CTP_SWAP(x, y)                 do{\
                                         typeof(x) z = x;\
                                         x = y;\
                                         y = z;\
                                       }while (0)

#endif /*_TOUCH_GTXXXX_H */
