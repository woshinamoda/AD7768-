/***************************************************************************//**
 *   @file   ad7768.c
 *   @brief  Implementation of AD7768 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2016(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//  	 AD7768数据口非SPI标准，无法直接用SPI从机卡DCLK下降沿缓存数据， 需要使用STM32 SAI串行时钟接口来读取
//     https://zhidao.baidu.com/question/1377847142233691779.htmlhttps://zhidao.baidu.com/question/1377847142233691779.html
//		 https://shequ.stmicroelectronics.cn/forum.php?mod=viewthread&tid=615182

//     https://m.elecfans.com/article/2008441.html
//     ADI 非标准SPI参考手册AN-1248，非常麻烦的使用spi读取数据，且需要程序开关，无法DMA

//  	application 文件夹中有官网下载源码 ad7768.c / ad7768.h


#include <stdio.h>
#include <stdlib.h>
#include "uApp_AD7768.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "spi.h"
#include "usart.h"
/* USER CODE END Includes */



/* Private ad7768 const code ----------------------------------------------------*/
/* USER CODE CONST BEGIN 0 */

const uint8_t standard_pin_ctrl_mode_sel[3][4] = {
//		MCLK/1,	MCLK/2,	MCLK/4,	MCLK/8
		{0x0,	0x1,	0x2,	0x3},	// Eco
		{0x4,	0x5,	0x6,	0x7},	// Median
		{0x8,	0x9,	0xA,	0xB},	// Fast
};

const uint8_t one_shot_pin_ctrl_mode_sel[3][4] = {
//		MCLK/1,	MCLK/2,	MCLK/4,	MCLK/8
		{0xC,	0xFF,	0xFF,	0xFF},	// Eco
		{0xD,	0xFF,	0xFF,	0xFF},	// Median
		{0xF,	0xE,	0xFF,	0xFF},	// Fast
};



/* USER CONST CODE END 0 */





/* AD7768 官方驱动代码 code ----------------------------------------------------*/
/* ad7768 function BEGIN 0 */

/**
 * SPI read from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_spi_read(ad7768_dev *dev,
						uint8_t reg_addr,
						uint8_t *reg_data)
{
	uint8_t buf[2];
	int32_t ret;

	buf[0] = 0x80 | (reg_addr & 0x7F);	//bit15 read=1
	buf[1] = 0x00;
	
	ret = spi_write_and_read(dev->spi_desc, buf, 2);
	
	//手册P50 off Frame protocol
	
	buf[0] = 0x80 | (reg_addr & 0x7F);
	buf[1] = 0x00;

	ret |= spi_write_and_read(dev->spi_desc, buf, 2);

	*reg_data = buf[1]; //P50 Figure 80. buf[0] = 0x00

	return ret;
}


/**
 * SPI write to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_spi_write(ad7768_dev *dev,
						 uint8_t reg_addr,
						 uint8_t reg_data)
{ //
	uint8_t buf[2];
	int32_t ret;

	buf[0] = (reg_addr & 0x7F);			//bit15	read=0
	buf[1] = reg_data;
	ret = spi_write_and_read(dev->spi_desc, buf, 2);

	return ret;
}

/**
 * SPI read from device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_spi_read_mask(ad7768_dev *dev,
							 uint8_t reg_addr,
							 uint8_t mask,
							 uint8_t *data)
{	//读取寄存器插入移位掩码
	uint8_t reg_data;
	int32_t ret;

	ret = ad7768_spi_read(dev, reg_addr, &reg_data);
	*data = (reg_data & mask);

	return ret;
}

/**
 * SPI write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_spi_write_mask(ad7768_dev *dev,
							  uint8_t reg_addr,
							  uint8_t mask,
							  uint8_t data)
{	//写入寄存器值插入移位掩码
	uint8_t reg_data;
	int32_t ret;

	ret = ad7768_spi_read(dev, reg_addr, &reg_data);		//先读对应寄存器值
	reg_data &= ~mask;																	//与移位值
	reg_data |= data;																		//或插入数据
	ret |= ad7768_spi_write(dev, reg_addr, reg_data);		//在写对应寄存器值

	return ret;
}

/**
 * Set the device sleep mode.
 * @param dev - The device structure.
 * @param mode - The device sleep mode.
 * 				 Accepted values: AD7768_ACTIVE
 * 								  AD7768_SLEEP
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_set_sleep_mode(ad7768_dev *dev,
							  ad7768_sleep_mode mode)
{	//设置睡眠模式
	ad7768_spi_write_mask(dev,
						  AD7768_REG_PWR_MODE,
						  AD7768_PWR_MODE_SLEEP_MODE,  
						  (mode ? AD7768_PWR_MODE_SLEEP_MODE : 0));
	dev->sleep_mode = mode;	//设置的模式写入状态结构体

	return 0;
}

/**
 * Get the device sleep mode.
 * @param dev - The device structure.
 * @param mode - The device sleep mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_get_sleep_mode(ad7768_dev *dev,
							  ad7768_sleep_mode *mode)
{ //获取设置睡眠状态
	*mode = dev->sleep_mode;

	return 0;
}


/**
 * Internal set MODEx pins function.
 * @param dev - The device structure.
 * @param state - The state of the MODEx pins.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_set_mode_pins(ad7768_dev *dev,
							 uint8_t state)
{ //设置AD7768 mode0/mode1/mode2/mode3 硬件引脚
	int32_t ret;

	if (dev->gpio_mode0 && dev->gpio_mode1 &&
			dev->gpio_mode2 && dev->gpio_mode3) {
		ret = gpio_set_value(dev->gpio_mode0,
						((state & 0x01) >> 0));
		ret |= gpio_set_value(dev->gpio_mode1,
						((state & 0x02) >> 1));
		ret |= gpio_set_value(dev->gpio_mode2,
						((state & 0x04) >> 2));
		ret |= gpio_set_value(dev->gpio_mode3,
						((state & 0x08) >> 3));
	} else {
		printf ("MODE GPIOs are not defined.");
		ret = -1;
	}

	return ret;
}



/**
 * Set the device power mode.
 * @param dev - The device structure.
 * @param mode - The device power mode.
 * 				 Accepted values: AD7768_ECO
 *								  AD7768_MEDIAN
 *								  AD7768_FAST
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_set_power_mode(ad7768_dev *dev,
							  ad7768_power_mode mode)
{ //设置AD7768功率
	uint8_t mode_pins_state;

	if (dev->pin_spi_ctrl == AD7768_SPI_CTRL) 
	{//如果是spi模式
		ad7768_spi_write_mask(dev,
							  AD7768_REG_PWR_MODE,
							  AD7768_PWR_MODE_POWER_MODE(0x3),
							  AD7768_PWR_MODE_POWER_MODE(mode));
		dev->power_mode = mode;
	}
	else
	{
		if (dev->conv_op == AD7768_STANDARD_CONV)
			mode_pins_state =
					standard_pin_ctrl_mode_sel[mode][dev->dclk_div];
		else
			mode_pins_state =
					one_shot_pin_ctrl_mode_sel[mode][dev->dclk_div];
		if (mode_pins_state != 0xFF) {
			dev->power_mode = mode;
			ad7768_set_mode_pins(dev, mode_pins_state);
		}
		else {
			printf("Invalid Power Mode for the current configuration.");

			return -1;
		}
	}
	return 0;
}


/**
 * Get the device power mode.
 * @param dev - The device structure.
 * @param mode - The device power mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_get_power_mode(ad7768_dev *dev,
							  ad7768_power_mode *mode)
{//获取当前电源模式
	*mode = dev->power_mode;

	return 0;
}


/**
 * Set the MCLK divider.
 * @param dev - The device structure.
 * @param clk_div - The MCLK divider.
 * 					Accepted values: AD7768_MCLK_DIV_32
 *									 AD7768_MCLK_DIV_8
 *									 AD7768_MCLK_DIV_4
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_set_mclk_div(ad7768_dev *dev,
							ad7768_mclk_div clk_div)
{ //设置当前MCLK时钟分频
	ad7768_spi_write_mask(dev,
						  AD7768_REG_PWR_MODE,
						  AD7768_PWR_MODE_MCLK_DIV(0x3),
						  AD7768_PWR_MODE_MCLK_DIV(clk_div));
	dev->mclk_div = clk_div;

	return 0;
}


/**
 * Get the MCLK divider.
 * @param dev - The device structure.
 * @param mode - The MCLK divider.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_get_mclk_div(ad7768_dev *dev,
							ad7768_mclk_div *clk_div)
{ //获取当前MCLK时钟分频
	*clk_div = dev->mclk_div;

	return 0;
}


/**
 * Set the DCLK divider.
 * @param dev - The device structure.
 * @param clk_div - The DCLK divider.
 * 					Accepted values: AD7768_DCLK_DIV_1
 *									 AD7768_DCLK_DIV_2
 *									 AD7768_DCLK_DIV_4
 *									 AD7768_DCLK_DIV_8
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_set_dclk_div(ad7768_dev *dev,
							ad7768_dclk_div clk_div)
{ //设置当前DCLK时钟
	uint8_t mode_pins_state;

	if (dev->pin_spi_ctrl == AD7768_SPI_CTRL) {
		ad7768_spi_write_mask(dev,
							  AD7768_REG_INTERFACE_CFG,
							  AD7768_INTERFACE_CFG_DCLK_DIV(0x3),
							  AD7768_INTERFACE_CFG_DCLK_DIV(clk_div));
		dev->dclk_div = clk_div;
	} else {
		if (dev->conv_op == AD7768_STANDARD_CONV)
			mode_pins_state =
					standard_pin_ctrl_mode_sel[dev->power_mode][clk_div];
		else
			mode_pins_state =
					one_shot_pin_ctrl_mode_sel[dev->power_mode][clk_div];
		if (mode_pins_state != 0xFF) {
			dev->dclk_div = clk_div;
			ad7768_set_mode_pins(dev, mode_pins_state);
		}
		else {
			printf("Invalid DCLK_DIV for the current configuration.");

			return -1;
		}
	}

	return 0;
}


/**
 * Get the DCLK divider.
 * @param dev - The device structure.
 * @param clk_div - The DCLK divider.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_get_dclk_div(ad7768_dev *dev,
							ad7768_dclk_div *clk_div)
{ //获取当前DCLK时钟
	*clk_div = dev->dclk_div;

	return 0;
}



/**
 * Set the conversion operation mode.
 * @param dev - The device structure.
 * @param conv_op - The conversion operation mode.
 * 					Accepted values: AD7768_STANDARD_CONV
 * 									 AD7768_ONE_SHOT_CONV
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_set_conv_op(ad7768_dev *dev,
						   ad7768_conv_op conv_op)
{ //数据转换 1次模式 or 标准模式
	uint8_t mode_pins_state;

	if (dev->pin_spi_ctrl == AD7768_SPI_CTRL) {
		ad7768_spi_write_mask(dev,
							  AD7768_REG_DATA_CTRL,
							  AD7768_DATA_CTRL_SINGLE_SHOT_EN,
							  conv_op ? AD7768_DATA_CTRL_SINGLE_SHOT_EN : 0);
		dev->conv_op = conv_op;
	} else {
		if (conv_op == AD7768_STANDARD_CONV)
			mode_pins_state =
					standard_pin_ctrl_mode_sel[dev->power_mode][dev->dclk_div];
		else
			mode_pins_state =
					one_shot_pin_ctrl_mode_sel[dev->power_mode][dev->dclk_div];
		if (mode_pins_state != 0xFF) {
			dev->conv_op = conv_op;
			ad7768_set_mode_pins(dev, mode_pins_state);
		}
		else {
			printf("Invalid Conversion Operation for the current configuration.");

			return -1;
		}
	}

	return 0;
}






/**
 * Get the conversion operation mode.
 * @param dev - The device structure.
 * @param conv_op - The conversion operation mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_get_conv_op(ad7768_dev *dev,
						   ad7768_conv_op *conv_op)
{ //获取数据转换模式
	*conv_op = dev->conv_op;

	return 0;
}




/**
 * Set the CRC selection.
 * @param dev - The device structure.
 * @param crc_sel - The CRC selection.
 * 					Accepted values: AD7768_NO_CRC
 * 									 AD7768_CRC_4
 * 									 AD7768_CRC_16
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_set_crc_sel(ad7768_dev *dev,
						   ad7768_crc_sel crc_sel)
{ //CRC校验选择
	ad7768_spi_write_mask(dev,
						  AD7768_REG_INTERFACE_CFG,
						  AD7768_INTERFACE_CFG_CRC_SEL(0x3),
						  AD7768_INTERFACE_CFG_CRC_SEL(crc_sel));
	dev->crc_sel = crc_sel;

	return 0;
}



/**
 * Get the CRC selection.
 * @param dev - The device structure.
 * @param crc_sel - The CRC selection.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_get_crc_sel(ad7768_dev *dev,
						   ad7768_crc_sel *crc_sel)
{ //获取CRC校验设置
	*crc_sel = dev->crc_sel;

	return 0;
}





/**
 * Set the channel state.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7768_CH0
 * 			   					AD7768_CH1
 * 			   					AD7768_CH2
 * 			   					AD7768_CH3
 * 			   					AD7768_CH4
 * 			   					AD7768_CH5
 * 			   					AD7768_CH6
 * 			   					AD7768_CH7
 * @param state - The channel state.
 * 				  Accepted values: AD7768_ENABLED
 * 								   AD7768_STANDBY
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_set_ch_state(ad7768_dev *dev,
							ad7768_ch ch,
							ad7768_ch_state state)
{ //选择通道设置状态
	ad7768_spi_write_mask(dev,
						  AD7768_REG_CH_STANDBY,
						  AD7768_CH_STANDBY(ch),
						  state ? AD7768_CH_STANDBY(ch) : 0);
	dev->ch_state[ch] = state;

	return 0;
}





/**
 * Get the channel state.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7768_CH0
 * 			   					AD7768_CH1
 * 			   					AD7768_CH2
 * 			   					AD7768_CH3
 * 			   					AD7768_CH4
 * 			   					AD7768_CH5
 * 			   					AD7768_CH6
 * 			   					AD7768_CH7
 * @param state - The channel state.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_get_ch_state(ad7768_dev *dev,
							ad7768_ch ch,
							ad7768_ch_state *state)
{ //获取通道状态
	*state = dev->ch_state[ch];

	return 0;
}



/**
 * Set the mode configuration.
 * @param dev - The device structure.
 * @param mode - The channel mode.
 * 				 Accepted values: AD7768_MODE_A
 * 								  AD7768_MODE_B
 * @param filt_type - The filter type.
 * 					  Accepted values: AD7768_FILTER_WIDEBAND
 * 					  				   AD7768_FILTER_SINC,
 * @param dec_rate - The decimation rate.
 * 					 Accepted values: AD7768_DEC_X32
 * 					 				  AD7768_DEC_X64
 * 					 				  AD7768_DEC_X128
 * 					 				  AD7768_DEC_X256
 * 					 				  AD7768_DEC_X512
 * 					 				  AD7768_DEC_X1024
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_set_mode_config(ad7768_dev *dev,
							   ad7768_ch_mode mode,
							   ad7768_filt_type filt_type,
							   ad7768_dec_rate dec_rate)
{ //设置数字滤波器 以及抽样率
	uint8_t reg_val;

	reg_val = ((filt_type == AD7768_FILTER_SINC) ? AD7768_CH_MODE_FILTER_TYPE : 0) |
			AD7768_CH_MODE_DEC_RATE(dec_rate);
	if (mode == AD7768_MODE_A) {
		ad7768_spi_write(dev, AD7768_REG_CH_MODE_A, reg_val);
	} else {
		ad7768_spi_write(dev, AD7768_REG_CH_MODE_B, reg_val);
	}
	dev->filt_type[mode] = filt_type;
	dev->dec_rate[mode] = dec_rate;

	return 0;
}




/**
 * Get the mode configuration.
 * @param dev - The device structure.
 * @param mode - The channel mode.
 * @param filt_type - The filter type.
 * @param dec_rate - The decimation rate.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_get_mode_config(ad7768_dev *dev,
							   ad7768_ch_mode mode,
							   ad7768_filt_type *filt_type,
							   ad7768_dec_rate *dec_rate)
{ //获取滤波器 以及 抽样率设置
	*filt_type = dev->filt_type[mode];
	*dec_rate = dev->dec_rate[mode];

	return 0;
}




/**
 * Set the channel mode.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7768_CH0
 * 			   					AD7768_CH1
 * 			   					AD7768_CH2
 * 			   					AD7768_CH3
 * 			   					AD7768_CH4
 * 			   					AD7768_CH5
 * 			   					AD7768_CH6
 * 			   					AD7768_CH7
 * @param mode - The channel mode.
 * 				 Accepted values: AD7768_MODE_A
 * 								  AD7768_MODE_B
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_set_ch_mode(ad7768_dev *dev,
						   ad7768_ch ch,
						   ad7768_ch_mode mode)
{ //设置通道模式 A or B
	ad7768_spi_write_mask(dev,
						  AD7768_REG_CH_MODE_SEL,
						  AD7768_CH_MODE(ch),
						  mode ? AD7768_CH_MODE(ch) : 0);
	dev->ch_mode[ch] = mode;

	return 0;
}



/**
 * Get the channel mode.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7768_CH0
 * 			   					AD7768_CH1
 * 			   					AD7768_CH2
 * 			   					AD7768_CH3
 * 			   					AD7768_CH4
 * 			   					AD7768_CH5
 * 			   					AD7768_CH6
 * 			   					AD7768_CH7
 * @param mode - The channel mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7768_get_ch_mode(ad7768_dev *dev,
						   ad7768_ch ch,
						   ad7768_ch_mode *mode)
{ //获取通道模式设置
	*mode = dev->ch_mode[ch];
	return 0;
}

/* ad7768 function END 0 */


/* 参考官方例程代码 ----------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t reg_data = 0;
uint8_t reg_addr = 9;
uint32_t sai_data[512] = {0};

ad7768_dev ad7768_device;						//ad7768配置声明
ad7768_init_param ad7768_init=			//ad7768初始化结构体声明
{
	.sleep_mode = AD7768_ACTIVE,
	.power_mode = AD7768_FAST,
	.mclk_div		= AD7768_MCLK_DIV_4,
	.dclk_div 	= AD7768_DCLK_DIV_1,
	.conv_op 		= AD7768_STANDARD_CONV,
	.crc_sel 		= AD7768_NO_CRC
};


int32_t ad7768_reset(void)
{
	ad7768_dev *dev = &ad7768_device;
	int32_t ret;
	uint8_t i;
	
	//用户自定义，尾旗标
	dev->tail = 0x1234;

	
	//AD7768 reset
  HAL_GPIO_WritePin(GPIOG, RESET_Pin, GPIO_PIN_RESET);	
	HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOG, RESET_Pin, GPIO_PIN_SET);	
	HAL_Delay(10);	

	//寄存器预设
	dev->pin_spi_ctrl = AD7768_SPI_CTRL;	
	dev->sleep_mode = AD7768_ACTIVE;
	dev->mclk_div = AD7768_MCLK_DIV_4;
	dev->crc_sel = AD7768_NO_CRC;

	if(dev->pin_spi_ctrl == AD7768_SPI_CTRL)
	{	//原理图SB3 连接，设置SPI控制
		ad7768_set_sleep_mode(dev, dev->sleep_mode);
		ad7768_set_mclk_div(dev, dev->mclk_div);
		ad7768_set_crc_sel(dev, dev->crc_sel);
	}

	dev->power_mode = AD7768_FAST;
	dev->dclk_div = AD7768_DCLK_DIV_1;
	dev->conv_op = AD7768_STANDARD_CONV;	
	ad7768_set_power_mode(dev, dev->power_mode);
	ad7768_set_dclk_div(dev, dev->dclk_div);
	ad7768_set_conv_op(dev, dev->conv_op);
	
	ad7768_set_power_mode(dev, dev->power_mode);
	ad7768_set_dclk_div(dev, dev->dclk_div);
	ad7768_set_conv_op(dev, dev->conv_op);
	
	ad7768_set_mode_config(dev, AD7768_MODE_A, AD7768_FILTER_SINC, AD7768_DEC_X32);	//设置通道A
	
	for(i=0; i<AD7768_8_CH; i++)
	{
		dev->ch_state[i] = AD7768_ENABLED;
		dev->gain[i] = 0x555555;		//P59:增益0x555555 为理想值 ， 数据转换P60
	}	
	
}


void ad7768_setup(void)
{
	uint8_t i;
	ad7768_dev *dev = &ad7768_device;
	
	if(ad7768_device.tail != 0x1234)
	{
		ad7768_reset();
	}	
	
	//AD7768 reset
  HAL_GPIO_WritePin(GPIOG, RESET_Pin, GPIO_PIN_RESET);	
	HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOG, RESET_Pin, GPIO_PIN_SET);	
	HAL_Delay(10);	
	
	ad7768_set_sleep_mode(dev, dev->sleep_mode);
	ad7768_set_mclk_div(dev, dev->mclk_div);
	ad7768_set_crc_sel(dev, dev->crc_sel);
	
	ad7768_set_power_mode(dev, dev->power_mode);
	ad7768_set_dclk_div(dev, dev->dclk_div);
	ad7768_set_conv_op(dev, dev->conv_op);
	
	ad7768_set_mode_config(dev, AD7768_MODE_A, dev->filt_type[AD7768_MODE_A], dev->dec_rate[AD7768_MODE_A]);
	
	for(i=0; i<AD7768_8_CH; i++)
	{

	}
}


/* USER CODE END 0 */







/* Private user function business code ----------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t reg_code[90];  //P73页，总计0x59寄存器

void my_ad7768_init()
{
	
//	//AD7768 reset
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);	
	HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);	
	HAL_Delay(10);	

	Fill_reg_List();		//reg_code 赋值,先读取是否默认寄存器配置
	CS_H;
	add7768_write_cmd(AD7768_REG_CH_STANDBY,			0x00);		//enable所有通道
	add7768_write_cmd(AD7768_REG_CH_MODE_A,				0x0D);		//默认A Sinc滤波器 x？？？采样率设置 ，请查阅手册P75
																													// x1024 Fast = 8		Khz采样率 0x0D  DCLK = MCLK/4 
																													// x512  Fast = 16	Khz采样率 0x0C
																													// x256  Fast = 32	Khz采样率 0x0B
																													// x128  Fast = 64	Khz采样率 0x0A
																													// x64   Fast = 128	Khz采样率 0x09	
																													// x32   Fast = 256	Khz采样率 0x08		
	
	
	
	
	add7768_write_cmd(AD7768_REG_CH_MODE_B,				0x0D);		//默认B Sinc滤波器 x？？？采样率设置
	add7768_write_cmd(AD7768_REG_CH_MODE_SEL,			0x00);		//默认所有通道选择A
	/*POWER MODE SELECT REGISTER*/	
	add7768_write_cmd(AD7768_REG_PWR_MODE,				0x00);		//bit7			|SLEEP_MODE		：0 Normal operation. 	1 Sleep mode.	
																													//bit[5:4]	|POWER_MODE		：00 Eco mode.  10 Median mode.  11 Fast mode.
																													//bit3		 	|LVDS_ENABLE	：0 LVDS input clock disabled.	 1 LVDS input clock enabled.
																													//bit[1:0] 	|MCLK_DIV			：00 MCLK/32:		10 MCLK/8:		11 MCLK/4:
	/*GENERAL DEVICE CONFIGURATION REGISTER*/	
	add7768_write_cmd(AD7768_REG_GENERAL_CFG,			0x22);		//bit5 			|RETIME_EN		：0 Disabled		1 Enable SYNC_OUT signal from MCLK	 
																													//bit4 			|VCM_PD				：0 Enable 			1 VCM Power Down 
																													//blt[1:0]	|VCM_VSEL			：00(AVDD1 - AVSS)/2 V.		01 1.65 V.		10 2.5V		11 2.14V  使用VCM时必须开启通道0
	/*DATA CONTROL: SOFT RESET, SYNC, AND SINGLE-SHOT CONTROL REGISTER*/	
	add7768_write_cmd(AD7768_REG_DATA_CTRL,				0x80);		//bit7			|SPI_SYNC			：0 SPI_SYNC low. 	1 SPI_SYNC high （只有1个设备默认高）
																													//bit4  		|SINGLE_SHOT	：0 Disabled.		1 Enabled.（不开启）
																													//blt[1:0]	|SPI_RESET		：No effect. 
	/*INTERFACE CONFIGURATION REGISTER*/
	add7768_write_cmd(AD7768_REG_INTERFACE_CFG,		0x00);	  //bit[3:2]	|CRC_SELECT		：00 No CRC
																													//blt[1:0]	|DCLK_DIV			：00 分频1/8		01 分频1/4		10 分频1/2		00 不分频（追求最大速率，不分频）  ！！DCLK最快8ns，还有增加采集频率空间，但不推荐


//	/*DIGITAL FILTER RAM BUILT IN SELF TEST (BIST) REGISTER*/
//	add7768_write_cmd(AD7768_REG_BIST_CTRL,				0x00);		// 内部RAM自检，不需要用到NC
//	/*STATUS REGISTER*/
//	add7768_write_cmd(AD7768_REG_DEV_STATUS,			0x00);		// 设备内部时钟 RAM状态检查， 该寄存器只能读取，无法写。 读取信息P79
//	/*Revision ID*/
//	add7768_write_cmd(AD7768_REG_REV_ID,					0x06);		// 设备ID for revisions details， 该寄存器只能读取，无法写。
//	/*GPIO CONTROL REGISTER*/
//	add7768_write_cmd(AD7768_REG_GPIO_CTRL,					0x00);		//bit7		|UGPIO_ENABLE	：0 GPIO Disable		1 GPIO Enable		GPIO放弃不用，后续关于GPIO的3个寄存器不做展示AD7768_REG_GPIO_WR_DATA | AD7768_REG_GPIO_RD_DATA
//																														//bit4		|GPIOE4_FILTER：0 input		1 output	
//																														//bit3		|GPIOE3_MODE3	：0 input		1 output
//																														//bit2		|GPIOE2_MODE2	：0 input		1 output	
//																														//bit1		|GPIOE1_MODE1	：0 input		1 output	
//																														//bit0		|GPIOE0_MODE0	：0 input		1 output	


	/*BUFFER ENABLE REGISTER 0 - 3*/
	add7768_write_cmd(AD7768_REG_PRECHARGE_BUF_1,		0xff);	//默认通道 0-3开启缓冲	
	/*BUFFER ENABLE REGISTER 4 - 7*/
	add7768_write_cmd(AD7768_REG_PRECHARGE_BUF_2,		0xff);	//默认通道 4-7开启缓冲	
	/*0-7负极参考缓冲*/
	add7768_write_cmd(AD7768_REG_POS_REF_BUF,				0x00);	//负极缓冲off		
	/*0-7正极参考缓冲*/
	add7768_write_cmd(AD7768_REG_NEG_REF_BUF,				0x00);	//正极缓冲off	
	
	
	for(uint8_t i=0; i<8; i++)
	{
		ad7768_gain_set(i+1, 0x555555);		//配置输出增益
	}
	

	Fill_reg_List();		//reg_code 读取设置后的寄存器校验
	
	//实际设置结果
	//MCLK = 4.2Mhz （1/4分频，实际输入晶振32.768M）
	//DCLK = 4.2Mhz
	//采集输入电源模式：FAST  单线D0只能采集8Mhz
}


uint8_t ad7768_read_cmd(uint8_t reg_addr)
{
	uint8_t set_buf[2];
	uint8_t read_buf[2];
	set_buf[0] = 0x80 | (reg_addr & 0x7F);	//reg_addr
	set_buf[1] = 0x00;											//None
	
	//CMD1,发送读取指令
	CS_L;
	HAL_SPI_TransmitReceive(&hspi1, set_buf, read_buf, 2, 0xff);
	CS_H;

	//CMD2，再次读取获取第一个偏移后的响应数据
	CS_L;
	HAL_SPI_TransmitReceive(&hspi1, set_buf, read_buf, 2, 0xff);
	CS_H;
	
	return read_buf[1];		//返回读取值
}

uint8_t add7768_write_cmd(uint8_t reg_addr, uint8_t data)
{
	uint8_t set_buf[2];
	uint8_t read_buf[2];
	
	set_buf[0] = (reg_addr & 0x7F);			//bit15	write=1
	set_buf[1] = data;						 			//reg data
	
	//CMD1,发送读取指令
	CS_L;
	HAL_SPI_TransmitReceive(&hspi1, set_buf, read_buf, 2, 0xff);
	CS_H;

	return set_buf[1]; //返回写入值	

}

void ad7768_gain_set(uint8_t chn, uint32_t gain)
{
	add7768_write_cmd(AD7768_REG_CH_GAIN_1(chn-1), (gain>>16) & 0x000000ff);	//MSB
	add7768_write_cmd(AD7768_REG_CH_GAIN_2(chn-1), (gain>>8) & 0x000000ff);		//Mid
	add7768_write_cmd(AD7768_REG_CH_GAIN_3(chn-1), (gain>>0) & 0x000000ff);		//LSB
}


void ad7768_start(void)
{//开启ADI采集


}


void ad7768_stop(void)
{//关闭ADI采集

}


void ad7768_rate_set(uint16_t rate)
{
	ad7768_device.rate = rate;
	
	if(rate <= 32)
	{
		ad7768_device.power_mode = AD7768_ECO;
		ad7768_device.mclk_div = AD7768_MCLK_DIV_32; 
	}
	else if(rate <= 128)
	{
		ad7768_device.power_mode = AD7768_FAST;
		ad7768_device.mclk_div = AD7768_MCLK_DIV_8; 
	}
	else
	{
		ad7768_device.power_mode = AD7768_FAST;
		ad7768_device.mclk_div = AD7768_MCLK_DIV_4; 
	}
	
	ad7768_set_power_mode(&ad7768_device, ad7768_device.power_mode);
	ad7768_set_mclk_div(&ad7768_device, ad7768_device.mclk_div);
	
	switch(rate)
	{
		case 1:
			ad7768_set_mode_config(&ad7768_device, AD7768_MODE_A, AD7768_FILTER_SINC, AD7768_DEC_X1024);
			break;
		case 2:
			ad7768_set_mode_config(&ad7768_device, AD7768_MODE_A, AD7768_FILTER_SINC, AD7768_DEC_X512);
			break;
		case 4:
			ad7768_set_mode_config(&ad7768_device, AD7768_MODE_A, AD7768_FILTER_SINC, AD7768_DEC_X256);
			break;
		case 8:
			ad7768_set_mode_config(&ad7768_device, AD7768_MODE_A, AD7768_FILTER_SINC, AD7768_DEC_X128);
			break;
		case 16:
			ad7768_set_mode_config(&ad7768_device, AD7768_MODE_A, AD7768_FILTER_SINC, AD7768_DEC_X64);
			break;
		case 32:
			ad7768_set_dclk_div(&ad7768_device, AD7768_DCLK_DIV_8);
			ad7768_set_mode_config(&ad7768_device, AD7768_MODE_A, AD7768_FILTER_SINC, AD7768_DEC_X32);
			break;
		case 64:
			ad7768_set_dclk_div(&ad7768_device, AD7768_DCLK_DIV_4);
			ad7768_set_mode_config(&ad7768_device, AD7768_MODE_A, AD7768_FILTER_SINC, AD7768_DEC_X64);
			break;
		case 128:
			ad7768_set_dclk_div(&ad7768_device, AD7768_DCLK_DIV_2);
			ad7768_set_mode_config(&ad7768_device, AD7768_MODE_A, AD7768_FILTER_SINC, AD7768_DEC_X32);
			break;
		case 256:
			ad7768_set_dclk_div(&ad7768_device, AD7768_DCLK_DIV_1);
			ad7768_set_mode_config(&ad7768_device, AD7768_MODE_A, AD7768_FILTER_SINC, AD7768_DEC_X32);
			break;
		
		default:
			break;
	}
}

void Fill_reg_List()
{//填充寄存器原始列表
	for(uint8_t i=0; i<0x59; i++)
	{
		reg_code[i]=ad7768_read_cmd(i);
		CS_H;		//速度太快？？有时候CS没拉高，后面一直读错
	}
}

/* USER CODE END 0 */





/* Private user interrupt function handle ----------------------------------------------------*/
/* USER HANDLE BEGIN 0 */


uint8_t ReadBuf[32];
int boardChannelDataInt[9] = {0xBBCC11FF};

void HAL_GPIO_EXTI_Callback(uint16_t	GPIO_Pin)
{
	if(GPIO_Pin == DRDY_Pin)
	{
		HAL_SPI_Receive(&hspi4, ReadBuf, 32, 0xff);

		for(uint8_t i=1; i<9; i++)
		{
			for (int j = 0; j < 3; j++)
			{ //  read 24 bits of channel data in 8 3 byte chunks
				uint8_t inByte;
				inByte = ReadBuf[j+1 + (i-1)*4];
				boardChannelDataInt[i] = (boardChannelDataInt[i] << 8) | inByte; // int data goes here
			}
		}

		
		for(uint8_t i=1; i<9; i++)
		{
			if ((boardChannelDataInt[i] & 0x00800000) == 0x00800000)
			{
				boardChannelDataInt[i] |= 0xFF000000;
			}else
			{
				boardChannelDataInt[i] &= 0x00FFFFFF;
			}
		}

		for(uint8_t i=1; i<9; i++)
		{
			boardChannelDataInt[i]  = boardChannelDataInt[i] * 0.4882817517;
		}	

		if(HAL_UART_Transmit(&huart3, boardChannelDataInt, 36,  1) != HAL_OK)
		{
			Error_Handler();
		}

	}

}






/* USER HANDLE END 0 */

































