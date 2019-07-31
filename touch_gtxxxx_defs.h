#ifndef _TOUCH_GTXXXX_DEFS_H
#define _TOUCH_GTXXXX_DEFS_H

#include <stdint.h>
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

//type definition
struct i2c_msg {
	uint16_t flags;	    /* control mark */
	uint16_t len;		/* Length of read and write data */
	uint8_t *buf;		/* pointer to store read and write data */
};

/* indicates read data */
#define CTP_I2C_M_RD		0x0001	

#define CTP_INT_TRIGGER             0

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


#define CTP_BAK_REF_SEND                0
#define CTP_BAK_REF_STORE               1
#define CFG_LOC_DRVA_NUM                29
#define CFG_LOC_DRVB_NUM                30
#define CFG_LOC_SENS_NUM                31

#define CTP_CHK_FW_MAX                  40
#define CTP_CHK_FS_MNT_MAX              300
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
	
#define CTP_DEBUG_ON         	0
// Log define
#define CTP_INFO(fmt,arg...)           printf("I> "fmt"\n",##arg)
#define CTP_ERROR(fmt,arg...)          printf("E> "fmt"\n",##arg)
#if CTP_DEBUG_ON>0 
#define CTP_DEBUG(fmt,arg...)          printf("D> [%d]"fmt"\n",__LINE__, ##arg);
#else
#define CTP_DEBUG(fmt,arg...)          
#endif


#ifndef NULL
  #define NULL        0
#endif

#endif //_TOUCH_GTXXXX_DEFS_H