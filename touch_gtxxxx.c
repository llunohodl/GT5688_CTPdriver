#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "touch_gtxxxx.h"
#include "main.h"

// GT9157 driver configuration
uint8_t CTP_CFG_GT9157[] ={ 
	0x00,0x20,0x03,0xE0,0x01,0x05,0x3C,0x00,0x01,0x08,
	0x28,0x0C,0x50,0x32,0x03,0x05,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x17,0x19,0x1E,0x14,0x8B,0x2B,0x0D,
	0x33,0x35,0x0C,0x08,0x00,0x00,0x00,0x9A,0x03,0x11,
	0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,
	0x00,0x20,0x58,0x94,0xC5,0x02,0x00,0x00,0x00,0x04,
	0xB0,0x23,0x00,0x93,0x2B,0x00,0x7B,0x35,0x00,0x69,
	0x41,0x00,0x5B,0x4F,0x00,0x5B,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,
	0x12,0x14,0x16,0x18,0x1A,0xFF,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0F,
	0x10,0x12,0x13,0x16,0x18,0x1C,0x1D,0x1E,0x1F,0x20,
	0x21,0x22,0x24,0x26,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
	0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0x48,0x01
};


// GT911 driver configuration
uint8_t CTP_CFG_GT911[] =  {
  0x00,0x20,0x03,0xE0,0x01,0x05,0x0D,0x00,0x01,0x08,
  0x28,0x0F,0x50,0x32,0x03,0x05,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8A,0x2A,0x0C,
  0x45,0x47,0x0C,0x08,0x00,0x00,0x00,0x02,0x02,0x2D,
  0x00,0x00,0x00,0x00,0x00,0x03,0x64,0x32,0x00,0x00,
  0x00,0x28,0x64,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,
  0x9C,0x2C,0x00,0x8F,0x34,0x00,0x84,0x3F,0x00,0x7C,
  0x4C,0x00,0x77,0x5B,0x00,0x77,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x18,0x16,0x14,0x12,0x10,0x0E,0x0C,0x0A,
  0x08,0x06,0x04,0x02,0xFF,0xFF,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x16,0x18,0x1C,0x1D,0x1E,0x1F,0x20,0x21,
  0x22,0x24,0x13,0x12,0x10,0x0F,0x0A,0x08,0x06,0x04,
  0x02,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x24,0x01	
};


// GT5688 driver configuration 5' 800*480 LCD 
//note that the first parameter of gt5688 should be 
//written as 0x97 to update the configuration.
uint8_t CTP_CFG_GT5688[]={ 
	0x97,0x20,0x03,0xE0,0x01,0x05,0x3D,0x10,0x01,0x00,
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

/* Touch IC type defaults*/
TOUCH_IC touchIC = GT9157;		

const TOUCH_PARAM_TypeDef touch_param[3] = 
{
  /* GT9157 */
  {
    .max_width = 800,
    .max_height = 480,
    .config_reg_addr = 0x8047,
  },
  /* GT911 */
  {
    .max_width = 800,
    .max_height = 480,
    .config_reg_addr = 0x8047,
  },
  /* GT5688  */
  {
    .max_width = 480,
    .max_height = 272,
    .config_reg_addr = 0x8050,
  }
};

static int8_t CTP_I2C_Test(void);

static void Delay(__IO uint32_t nCount){
	for(; nCount != 0; nCount--);
}

#ifndef NULL
  #define NULL        0
#endif
/* indicates read data */
#define CTP_I2C_M_RD		0x0001	

#define CTP_ADDRESS      0xBA //0xBA or 0x28

#define CTP_GPIO_RCC_ENABLE   __HAL_RCC_GPIOH_CLK_ENABLE();
#define CTP_RST_PIN           GPIO_PIN_4
#define CTP_RST_PORT          GPIOB
#define CTP_INT_PIN           GPIO_PIN_5
#define CTP_INT_PORT          GPIOB

void I2C_ResetChip(void){
    GPIO_InitTypeDef GPIO_Initure;
    //1. Reset out low
    CTP_RST_PORT->BSRR=CTP_RST_PIN<<16; //RST = 0
	HAL_Delay(5);
    //2. Int out low / high
    GPIO_Initure.Pin=CTP_INT_PIN;            
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;             
	HAL_GPIO_Init(CTP_INT_PORT,&GPIO_Initure);
    #if CTP_ADDRESS >= 0xBA
      CTP_INT_PORT->BSRR=CTP_INT_PIN<<16; //INT = 0
    #else
      CTP_INT_PORT->BSRR=CTP_INT_PIN<<0;  //INT = 1
    #endif
    HAL_Delay(2); //>100us
    //3. Reset out high
	CTP_RST_PORT->BSRR=CTP_RST_PIN<<0;  //RST = 1
	HAL_Delay(7); //5-10 ms
    //4. INT out low
    CTP_INT_PORT->BSRR=CTP_INT_PIN<<16; //INT = 0
    HAL_Delay(55); //>50 ms
    //5. INT input floating
	GPIO_Initure.Pin=CTP_INT_PIN;            
	GPIO_Initure.Mode=GPIO_MODE_INPUT;              
	HAL_GPIO_Init(CTP_INT_PORT,&GPIO_Initure);
}

void I2C_Touch_Init(void){
    GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();		//Turn on the GPIOH clock
	
    GPIO_Initure.Pin=CTP_RST_PIN;            
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;
    GPIO_Initure.Pull=GPIO_NOPULL;
	HAL_GPIO_Init(CTP_RST_PORT,&GPIO_Initure);
    CTP_RST_PORT->BSRR=CTP_RST_PIN<<16; //RST = 0
    HAL_Delay(10);

    I2C_ResetChip();
    HAL_Delay(50);
    MX_I2C1_Init();
}

/**
  * @brief uses IIC for data transfer
  * @param
  * @arg i2c_msg: data transfer structure
  * @arg num: number of data transfer structures
  * @retval The number of transmission structures completed normally, if not normal, return 0xff
  */
static int I2C_Transfer( struct i2c_msg *msgs,int num){
	int im = 0;
	int ret = 0;

	CTP_DEBUG_FUNC();

	for (im = 0; ret == 0 && im != num; im++){
		if ((msgs[im].flags&CTP_I2C_M_RD)){
			HAL_I2C_Master_Receive(&hi2c1,CTP_ADDRESS+CTP_I2C_M_RD /*msgs[im].addr*/, msgs[im].buf, msgs[im].len,15+msgs[im].len/10); 
            ret=0;
		}else{
			HAL_I2C_Master_Transmit(&hi2c1,CTP_ADDRESS /*msgs[im].addr*/,  msgs[im].buf, msgs[im].len,15+msgs[im].len/10); 
            ret=0;
		}
	}

	if(ret)
		return ret;

	return im;   													
}

/**
  * @brief reads data from IIC devices
  * @param
  * @arg client_addr: device address
  * @arg buf[0~1]: Read the start address of the data register
  * @arg buf[2~len-1]: store buffer buffer for reading out data
  * @arg len: CTP_ADDR_LENGTH + read bytes count (register address length + number of data bytes read)
  * @retval i2c_msgs the number of transport structures, 2 is successful, others are failed
  */
static int32_t CTP_I2C_Read(uint8_t client_addr, uint8_t *buf, int32_t len){
    struct i2c_msg msgs[2];
    int32_t ret=-1;
    int32_t retries = 0;

    CTP_DEBUG_FUNC();
    msgs[0].flags = !CTP_I2C_M_RD;					
    msgs[0].addr  = client_addr;
    msgs[0].len   = CTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    
    msgs[1].flags = CTP_I2C_M_RD;
    msgs[1].addr  = client_addr;
    msgs[1].len   = len - CTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[CTP_ADDR_LENGTH];

    while(retries < 5){
        ret = I2C_Transfer( msgs, 2);					
        if(ret == 2)break;
        retries++;
    }
    if((retries >= 5)){
        CTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((uint16_t)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}

/**
  * @brief writes data to IIC device
  * @param
  * @arg client_addr: device address
  * @arg buf[0~1]: start address of the data register to be written
  * @arg buf[2~len-1]: data to be written
  * @arg len: CTP_ADDR_LENGTH + write bytes count (register address length + number of data bytes written)
  * @retval i2c_msgs the number of transport structures, 1 is successful, others are failed
  */
static int32_t CTP_I2C_Write(uint8_t client_addr,uint8_t *buf,int32_t len){
    struct i2c_msg msg;
    int32_t ret = -1;
    int32_t retries = 0;

    CTP_DEBUG_FUNC();
    msg.flags = !CTP_I2C_M_RD;			
    msg.addr  = client_addr;		
    msg.len   = len;					
    msg.buf   = buf;					
    while(retries < 5){
        ret = I2C_Transfer(&msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5)){
      CTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((uint16_t)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}

/**
  * @brief uses IIC to read the data again and check if it is normal.
  * @param
  * @arg client: device address
  * @arg addr: register address
  * @arg rxbuf: store the read data
  * @arg len: number of bytes read
  * @retval
  * @arg FAIL
  * @arg SUCCESS
  */
int32_t CTP_I2C_Read_dbl_check(uint8_t client_addr, uint16_t addr, uint8_t *rxbuf, int len){
    uint8_t buf[16] = {0};
    uint8_t confirm_buf[16] = {0};
    uint8_t retry = 0;
    
    CTP_DEBUG_FUNC();

    while (retry++ < 3){
        memset(buf, 0xAA, 16);
        buf[0] = (uint8_t)(addr >> 8);
        buf[1] = (uint8_t)(addr & 0xFF);
        CTP_I2C_Read(client_addr, buf, len + 2);
        
        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (uint8_t)(addr >> 8);
        confirm_buf[1] = (uint8_t)(addr & 0xFF);
        CTP_I2C_Read(client_addr, confirm_buf, len + 2);

        if (!memcmp(buf, confirm_buf, len+2)){
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }    
    CTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
    return FAIL;
}

/**
  * @brief turns off GT91xx interrupt
  * @param none
  * @retval none
  */
void CTP_IRQ_Disable(void){
    CTP_DEBUG_FUNC();
    //I2C_CTP_IRQDisable();
}

/**
  * @brief Enable GT91xx interrupt
  * @param none
  * @retval none
  */
void CTP_IRQ_Enable(void){
	CTP_DEBUG_FUNC();
	//I2C_CTP_IRQEnable();    
}

int16_t pre_x=-1;
int16_t pre_y=-1;

/**
  * @brief process or report touch screen detection to press
  * @param
  * @arg id: touch order trackID
  * @arg x: the x coordinate of the touch
  * @arg y: the y coordinate of the touch
  * @arg w: the size of the touch
  * @retval none
  */
static void CTP_Touch_Down(int32_t x,int32_t y,int32_t w){
	CTP_DEBUG_FUNC();
    /* Take x, y initial value is greater than the screen pixel value */
    CTP_DEBUG("X:%d, Y:%d, W:%d",  x, y, w);
    pre_x = x; pre_y=y;
}

/**
  * @brief process or report touch screen release
  * @param release point id number
  * @retval none
  */
static void CTP_Touch_Up(void){   // touch screen output without carton
    /* stylus release, reset pre xy to negative */
	static int bef_x = -1, bef_y = -1;
	if(pre_x == bef_x && pre_y == bef_y) return;
	bef_x = pre_x;
	bef_y = pre_y;
	pre_x = -1;
	pre_y = -1;  
    CTP_DEBUG("Touch release!");
}

/**
  * @brief touch screen processing function, polling or calling in touch interrupt
  * @param none
  * @retval none
  */
static void Goodix_TS_Work_Func(void){
	static uint8_t IsTouch=0;
	uint8_t  end_cmd[3] = {CTP_READ_COOR_ADDR >> 8, CTP_READ_COOR_ADDR & 0xFF, 0};
	uint8_t  point_data[2 + 1 + 8 * CTP_MAX_TOUCH + 1]={CTP_READ_COOR_ADDR >> 8, CTP_READ_COOR_ADDR & 0xFF};
	uint8_t  touch_num = 0;
	uint8_t  finger = 0;

	uint8_t client_addr=CTP_ADDRESS;
	int32_t input_x = 0;
	int32_t input_y = 0;
	int32_t input_w = 0;
	int32_t ret = -1;
	
	CTP_DEBUG_FUNC();
	
	ret = CTP_I2C_Read(client_addr, point_data, 12);//10 byte register plus 2 byte address
	if (ret < 0){
        CTP_ERROR("I2C transfer error. errno:%d\n ", ret);
        return;
	}
	
	finger = point_data[CTP_ADDR_LENGTH]; // Status register data

	CTP_DEBUG("I2C finger:%X",finger);

	if(finger == 0x00){		// No data, exit
		if(IsTouch){
			CTP_Touch_Up();
			IsTouch=0;
		}
		return;
	}

	if((finger & 0x80) != 0){ 
		touch_num = finger & 0x0f; // coordinate points
	}

	if (touch_num <= CTP_MAX_TOUCH){
      IsTouch=1;
      input_x  = point_data[4] | (point_data[5] << 8);	//x coordinates
      input_y  = point_data[6] | (point_data[7] << 8);	//y coordinates
      input_w  = point_data[8] | (point_data[9] << 8);	//size

      CTP_Touch_Down(input_x, input_y, input_w);          //data processing
    }

    ret = CTP_I2C_Write(client_addr, end_cmd, 3);
    if (ret < 0){
        CTP_INFO("I2C write end_cmd error!");
    }
}


/**
  * @brief resets the touch screen chip
  * @param none
  * @retval none
  */
int8_t CTP_Reset_Guitar(void){
    CTP_DEBUG_FUNC();
#if 1
    I2C_ResetChip();
    return 0;
#else 		//Software reset
    int8_t ret = -1;
    int8_t retry = 0;
    uint8_t reset_command[3]={(uint8_t)CTP_REG_COMMAND>>8,(uint8_t)CTP_REG_COMMAND&0xFF,2};

    // Write reset command
    while(retry++ < 5){
        ret = CTP_I2C_Write(CTP_ADDRESS, reset_command, 3);
        if (ret > 0){
            CTP_INFO("GTP enter sleep!");
            return ret;
        }
    }
    CTP_ERROR("GTP send sleep cmd failed.");
    return ret;
#endif
}


int8_t CTP_Send_Command(uint8_t command){
    int8_t ret = -1;
    int8_t retry = 0;
    uint8_t command_buf[3] = {(uint8_t)(CTP_REG_COMMAND >> 8), (uint8_t)CTP_REG_COMMAND&0xFF, CTP_COMMAND_READSTATUS};

    CTP_DEBUG_FUNC();

    while(retry++ < 5){
        ret = CTP_I2C_Write(CTP_ADDRESS, command_buf, 3);
        if (ret > 0){
            CTP_INFO("send command success!");
            return ret;
        }

    }
    CTP_ERROR("send command fail!");
    return ret;
}

/**
  * @brief wake up touch screen
  * @param none
  * @retval 0 is successful, others are failed
  */
int8_t CTP_WakeUp_Sleep(void){
    uint8_t retry = 0;
    int8_t ret = -1;

    CTP_DEBUG_FUNC();

    while(retry++ < 10){
        ret = CTP_I2C_Test();
        if (ret > 0){
            CTP_INFO("GTP wakeup sleep.");
            return ret;
        }
        CTP_Reset_Guitar();
    }

    CTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}

static int32_t CTP_Get_Info(void){
    uint8_t opr_buf[10] = {0};
    int32_t ret = 0;

    uint16_t abs_x_max = CTP_MAX_WIDTH;
    uint16_t abs_y_max = CTP_MAX_HEIGHT;
    uint8_t int_trigger_type = CTP_INT_TRIGGER;
        
    opr_buf[0] = (uint8_t)((CTP_REG_CONFIG_DATA+1) >> 8);
    opr_buf[1] = (uint8_t)((CTP_REG_CONFIG_DATA+1) & 0xFF);
    
    ret = CTP_I2C_Read(CTP_ADDRESS, opr_buf, 10);
    if (ret < 0){
        return FAIL;
    }
    
    abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
    abs_y_max = (opr_buf[5] << 8) + opr_buf[4];
		CTP_DEBUG("RES");   
		CTP_DEBUG_ARRAY(&opr_buf[0],10);

    opr_buf[0] = (uint8_t)((CTP_REG_CONFIG_DATA+6) >> 8);
    opr_buf[1] = (uint8_t)((CTP_REG_CONFIG_DATA+6) & 0xFF);
    
    ret = CTP_I2C_Read(CTP_ADDRESS, opr_buf, 3);
    if (ret < 0){
        return FAIL;
    }
    int_trigger_type = opr_buf[2] & 0x03;
    
    CTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
            abs_x_max,abs_y_max,int_trigger_type);
    
    return SUCCESS;    
}


int32_t CTP_Init_Panel(void)
{
    int32_t ret = -1;

    int32_t i = 0;
    uint16_t check_sum = 0;
    int32_t retry = 0;

    const uint8_t* cfg_info;
    uint8_t cfg_info_len  ;
		uint8_t* config;

    uint8_t cfg_num =0 ;// The number of registers that need to be configured

    CTP_DEBUG_FUNC();


    I2C_Touch_Init();

    ret = CTP_I2C_Test();
    if (ret < 0){
        CTP_ERROR("I2C communication ERROR!");
		return ret;
    } 
    HAL_Delay(1000);
    CTP_Read_Version(); // Get the model of the touch IC

    config = (uint8_t *)malloc (CTP_CONFIG_MAX_LENGTH + CTP_ADDR_LENGTH);

    config[0] = CTP_REG_CONFIG_DATA >> 8;
    config[1] =  CTP_REG_CONFIG_DATA & 0xff;
    
    // According to the model of the IC points to a different configuration
    switch(touchIC){
    case GT9157:
        cfg_info =  CTP_CFG_GT9157; // Point to register configuration
        cfg_info_len = CFG_GROUP_LEN(CTP_CFG_GT9157);// Calculate the size of the configuration table
        break;
    case GT911:
        cfg_info =  CTP_CFG_GT911;
        cfg_info_len = CFG_GROUP_LEN(CTP_CFG_GT911) ;
        break;
    case GT5688:			
        cfg_info =  CTP_CFG_GT5688; 
        cfg_info_len = CFG_GROUP_LEN(CTP_CFG_GT5688);
        break;
    }
	
    memset(&config[CTP_ADDR_LENGTH], 0, CTP_CONFIG_MAX_LENGTH);
    memcpy(&config[CTP_ADDR_LENGTH], cfg_info, cfg_info_len);
		
    cfg_num = cfg_info_len;
    
    CTP_DEBUG("cfg_info_len = %d ",cfg_info_len);
    CTP_DEBUG("cfg_num = %d ",cfg_num);
    CTP_DEBUG_ARRAY(config,6);
    
    /*Set the resolution according to the scanning direction of the LCD*/
    config[CTP_ADDR_LENGTH+1] = LCD_PIXEL_WIDTH & 0xFF;
    config[CTP_ADDR_LENGTH+2] = LCD_PIXEL_WIDTH >> 8;
    config[CTP_ADDR_LENGTH+3] = LCD_PIXEL_HEIGHT & 0xFF;
    config[CTP_ADDR_LENGTH+4] = LCD_PIXEL_HEIGHT >> 8;
  
    /*Set X2Y exchange according to mode*/  
    //exchange
    config[CTP_ADDR_LENGTH+(0x8056-0x8050)-1] |= (X2Y_LOC);

    // Calculate the value to be written to the checksum register
    check_sum = 0;
    if(touchIC == GT911 || touchIC == GT9157){
        for (i = CTP_ADDR_LENGTH; i < cfg_num+CTP_ADDR_LENGTH; i++){
            check_sum += (config[i] & 0xFF);
        }
        config[ cfg_num+CTP_ADDR_LENGTH] = (~(check_sum & 0xFF)) + 1; 	//checksum
        config[ cfg_num+CTP_ADDR_LENGTH+1] =  1; 						//refresh Configuration update flag
    }
    else if(touchIC == GT5688) {
      for (i = CTP_ADDR_LENGTH; i < (cfg_num+CTP_ADDR_LENGTH -3); i += 2){
        check_sum += (config[i] << 8) + config[i + 1];
      }
      check_sum = 0 - check_sum;
      CTP_DEBUG("Config checksum: 0x%04X", check_sum);
      config[(cfg_num+CTP_ADDR_LENGTH -3)] = (check_sum >> 8) & 0xFF;
      config[(cfg_num+CTP_ADDR_LENGTH -2)] = check_sum & 0xFF;
      config[(cfg_num+CTP_ADDR_LENGTH -1)] = 0x01;
    }

    
    // Write configuration information
    for (retry = 0; retry < 5; retry++){
        ret = CTP_I2C_Write(CTP_ADDRESS, config , cfg_num + CTP_ADDR_LENGTH+2);
        if (ret > 0){
            break;
        }
    }
    Delay(0xfffff);		
		
#if 1	
    uint8_t buf[300];
    buf[0] = config[0];
    buf[1] =config[1];    
    CTP_DEBUG_FUNC();
    ret = CTP_I2C_Read(CTP_ADDRESS, buf, sizeof(buf));
    CTP_DEBUG("read ");
    CTP_DEBUG_ARRAY(buf,cfg_num);
    CTP_DEBUG("write ");
    CTP_DEBUG_ARRAY(config,cfg_num);
    for(i=3;i<cfg_num+CTP_ADDR_LENGTH-3;i++){
        if(config[i] != buf[i]){
            CTP_ERROR("Config fail ! i = %d ",i);
            free(config);
            return -1;
        }
    }
    if(i==cfg_num+CTP_ADDR_LENGTH-3)
        CTP_DEBUG("Config success ! i = %d ",i);
#endif
	
    CTP_Get_Info();	
	free(config);
    
    CTP_WakeUp_Sleep();
    return 0;
}

int32_t CTP_Read_Version(void){
    int32_t ret = -1;
    uint8_t buf[8] = {CTP_REG_VERSION >> 8, CTP_REG_VERSION & 0xff};    //¼Ä´æÆ÷µØÖ·

    CTP_DEBUG_FUNC();

    ret = CTP_I2C_Read(CTP_ADDRESS, buf, sizeof(buf));
    if (ret < 0){
        CTP_ERROR("GTP read version failed");
        return ret;
    }

    if (buf[2] == '9'){				
      //GT911
      if(buf[2] == '9' && buf[3] == '1' && buf[4] == '1'){
        CTP_INFO("IC1 Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
        touchIC = GT911;
      }
      //GT9157
      else if( buf[2] == '9' && buf[3] == '1' && buf[4] == '5' && buf[5] == '7'){
        CTP_INFO("IC2 Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
        touchIC = GT9157;
      }else{
         CTP_INFO("Unknown IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
      }
    }    
    else if (buf[2] == '5'){	
      //GT5688
      if(buf[2] == '5' && buf[3] == '6' && buf[4] == '8' && buf[5] == '8'){
        CTP_INFO("IC3 Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
        touchIC = GT5688;
      }else{
           CTP_INFO("Unknown IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
      }

    } else {
       CTP_INFO("Unknown IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    return ret;
}

static int8_t CTP_I2C_Test( void){
    uint8_t test[3] = {CTP_REG_CONFIG_DATA >> 8, CTP_REG_CONFIG_DATA & 0xff};
    uint8_t retry = 0;
    int8_t ret = -1;

    CTP_DEBUG_FUNC();
  
    while(retry++ < 5){
        ret = CTP_I2C_Read(CTP_ADDRESS, test, 3);
        if (ret > 0){
            return ret;
        }
        CTP_ERROR("GTP i2c test failed time %d.",retry);
    }
    return ret;
}

void CTP_TouchProcess(void){
  CTP_DEBUG_FUNC();
  Goodix_TS_Work_Func();
}

#if 0
static void GT91xx_Config_Read_Proc(void)
{
    char temp_data[CTP_CONFIG_MAX_LENGTH + 2] = {0x80, 0x47};
    int i;

    CTP_INFO("==== GT9XX config init value====\n");

    for (i = 0 ; i < CTP_CONFIG_MAX_LENGTH ; i++){
        printf("reg0x%x = 0x%02X ", i+0x8047, config[i + 2]);
        if (i % 10 == 9)
            printf("\n");
    }

    CTP_INFO("==== GT9XX config real value====\n");
    CTP_I2C_Read(CTP_ADDRESS, (uint8_t *)temp_data, CTP_CONFIG_MAX_LENGTH + 2);
    for (i = 0 ; i < CTP_CONFIG_MAX_LENGTH ; i++){
        printf("reg0x%x = 0x%02X ", i+0x8047,temp_data[i+2]);
        if (i % 10 == 9)
            printf("\n");
    }

}

static int32_t GT91xx_Config_Write_Proc(void)
{
    int32_t ret = -1;

    int32_t i = 0;
    uint8_t check_sum = 0;
    int32_t retry = 0;
    uint8_t cfg_num =0x80FE-0x8047+1 ;		

    uint8_t cfg_info[] = CTP_CFG_GROUP1;
    uint8_t cfg_info_len =CFG_GROUP_LEN(cfg_info) ;

    CTP_INFO("==== GT9XX send config====\n");

    memset(&config[CTP_ADDR_LENGTH], 0, CTP_CONFIG_MAX_LENGTH);
    memcpy(&config[CTP_ADDR_LENGTH], cfg_info,cfg_info_len);

    check_sum = 0;
    for (i = CTP_ADDR_LENGTH; i < cfg_num+CTP_ADDR_LENGTH; i++){
        check_sum += config[i];
    }
    config[ cfg_num+CTP_ADDR_LENGTH] = (~check_sum) + 1; 	
    config[ cfg_num+CTP_ADDR_LENGTH+1] =  1; 						

    for (retry = 0; retry < 5; retry++){
        ret = CTP_I2C_Write(CTP_ADDRESS, config , cfg_num + CTP_ADDR_LENGTH+2);
        if (ret > 0){
            break;
        }
    }


    return ret;
}

#endif

void CTP_touch_polling(){
  if((GPIOB->IDR&(1<<5))==0){
    CTP_TouchProcess(); 
  }
}

#if 0
void CTP_IRQHandler(void){
	if(__HAL_GPIO_EXTI_GET_IT(CTP_INT_GPIO_PIN) != RESET){
        CTP_TouchProcess();    
        __HAL_GPIO_EXTI_CLEAR_IT(CTP_INT_GPIO_PIN);     
	}  
}
#endif

