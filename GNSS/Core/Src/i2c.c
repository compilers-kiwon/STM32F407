/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "lsm6dsox_reg.h"
#include "lis2mdl_reg.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
/* I2C3 init function */
void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* I2C3 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(Audio_SCL_GPIO_Port, Audio_SCL_Pin);

    HAL_GPIO_DeInit(Audio_SDA_GPIO_Port, Audio_SDA_Pin);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static lsm6dsox_ctx_t lsm6dsox_ctx;
static lis2mdl_ctx_t  lis2mdl_ctx;

static void platform_delay(uint32_t ms)
{
	HAL_Delay(ms);
}

static int32_t	LSM6DSOX_register_read(void* ctx,uint8_t reg_addr,uint8_t *data,size_t len)
{
    uint8_t     write_buffer[1] = {reg_addr};

    if( HAL_I2C_Master_Transmit(&hi2c3,LSM6DSOX_ADDR,write_buffer,1,30)!=HAL_OK
    			|| HAL_I2C_Master_Receive(&hi2c3,LSM6DSOX_ADDR,data,len,30)!=HAL_OK )
    {
        printf("Cannot access LSM6DSOX sensor!!\n");
        return	HAL_ERROR;
    }

    return  HAL_OK;
}

static int32_t	LSM6DSOX_register_write(void* ctx,uint8_t reg_addr,const uint8_t *data,size_t len)
{
	uint8_t	write_buffer[100];

	write_buffer[0] = reg_addr;
	memcpy(&write_buffer[1],data,len);

	return	HAL_I2C_Master_Transmit(&hi2c3,LSM6DSOX_ADDR,write_buffer,len+1,50);
}

static int32_t	LIS2MDL_register_read(void *ctx,uint8_t reg,uint8_t *data,size_t len)
{
    lsm6dsox_sh_cfg_read_t    sh_cfg_read;
    lsm6dsox_status_master_t  master_status;

    int16_t data_raw_acceleration[3];
    int32_t ret;
    uint8_t drdy;

    /* Disable accelerometer. */
    lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx,LSM6DSOX_XL_ODR_OFF);

    /* Configure Sensor Hub to read LIS2MDL. */
    sh_cfg_read.slv_add = (LIS2MDL_I2C_ADD&0xFEU)>>1; /* 7bit I2C address */
    sh_cfg_read.slv_subadd = reg;
    sh_cfg_read.slv_len = len;

    ret = lsm6dsox_sh_slv0_cfg_read(&lsm6dsox_ctx,&sh_cfg_read);
    lsm6dsox_sh_slave_connected_set(&lsm6dsox_ctx,LSM6DSOX_SLV_0);

    /* Enable I2C Master and I2C master. */
    lsm6dsox_sh_master_set(&lsm6dsox_ctx,PROPERTY_ENABLE);

    /* Enable accelerometer to trigger Sensor Hub operation. */
    lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx,LSM6DSOX_XL_ODR_104Hz);

    /* Wait Sensor Hub operation flag set. */
    lsm6dsox_acceleration_raw_get(&lsm6dsox_ctx,data_raw_acceleration);

    do {
        platform_delay(20);
        lsm6dsox_xl_flag_data_ready_get(&lsm6dsox_ctx,&drdy);
    } while (!drdy);

    do {
        platform_delay(20);
        lsm6dsox_sh_status_get(&lsm6dsox_ctx,&master_status);
    } while (!master_status.sens_hub_endop);

    /* Disable I2C master and XL(trigger). */
    lsm6dsox_sh_master_set(&lsm6dsox_ctx,PROPERTY_DISABLE);
    lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx,LSM6DSOX_XL_ODR_OFF);

    /* Read SensorHub registers. */
    lsm6dsox_sh_read_data_raw_get(&lsm6dsox_ctx,(lsm6dsox_emb_sh_read_t*)data,len);

    return ret;
}

static int32_t	LIS2MDL_register_write(void *ctx,uint8_t reg,const uint8_t *data,size_t len)
{
    int16_t data_raw_acceleration[3];
    int32_t ret;
    uint8_t drdy;

    lsm6dsox_status_master_t  master_status;
    lsm6dsox_sh_cfg_write_t   sh_cfg_write;

    /* Configure Sensor Hub to read LIS2MDL. */
    sh_cfg_write.slv0_add = (LIS2MDL_I2C_ADD&0xFEU)>>1; /* 7bit I2C address */
    sh_cfg_write.slv0_subadd = reg,
    sh_cfg_write.slv0_data = *data,
    ret = lsm6dsox_sh_cfg_write(&lsm6dsox_ctx,&sh_cfg_write);

    /* Disable accelerometer. */
    lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx,LSM6DSOX_XL_ODR_OFF);

    /* Enable I2C Master. */
    lsm6dsox_sh_master_set(&lsm6dsox_ctx,PROPERTY_ENABLE);

    /* Enable accelerometer to trigger Sensor Hub operation. */
    lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx,LSM6DSOX_XL_ODR_104Hz);

    /* Wait Sensor Hub operation flag set. */
    lsm6dsox_acceleration_raw_get(&lsm6dsox_ctx,data_raw_acceleration);

    do {
        platform_delay(20);
        lsm6dsox_xl_flag_data_ready_get(&lsm6dsox_ctx,&drdy);
    } while (!drdy);

    do {
        platform_delay(20);
        lsm6dsox_sh_status_get(&lsm6dsox_ctx,&master_status);
    } while (!master_status.sens_hub_endop);

    /* Disable I2C master and XL (trigger). */
    lsm6dsox_sh_master_set(&lsm6dsox_ctx,PROPERTY_DISABLE);
    lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx,LSM6DSOX_XL_ODR_OFF);

    return ret;
}

static int32_t	initialize_imu_device(void)
{
    lsm6dsox_sh_cfg_read_t  sh_cfg_read;
    lsm6dsox_reg_t          reg;
    int32_t                 ret;

    /* Wait sensor boot time */
    platform_delay(20);

    /* Check lsm6dsox ID. */
    reg.byte = 0;

    for(int i=0;i<10;i++)
    {
        lsm6dsox_device_id_get(&lsm6dsox_ctx,&reg.byte);
        if( reg.byte != 0 )break;
        platform_delay(10);
    }

    if( reg.byte == 0 )
    {
        printf("Can't detect LSM6DSOX sensor!\n");
        return  HAL_ERROR;
    }
    else
    {
    	printf("LSM6DSOX WHO_AM_I = %X\n",reg.byte);
    }

    /* Restore default configuration. */
    if( lsm6dsox_reset_set(&lsm6dsox_ctx,PROPERTY_ENABLE) != 0 )
    {
        printf("Can't restore default configuration!\n");
        return  HAL_ERROR;
    }

    for(uint32_t i=0;;i++)
    {
        if( i == 10 )
        {
            printf("Can't confirm default configuration!\n");
            return  HAL_ERROR;
        }

        if( lsm6dsox_reset_get(&lsm6dsox_ctx,&reg.byte) == HAL_OK )
        {
            break;
        }
    }

    /* Disable I3C interface.*/
    if( lsm6dsox_i3c_disable_set(&lsm6dsox_ctx,LSM6DSOX_I3C_DISABLE) != 0 )
    {
        printf("Can't disable I3C interface!\n");
        return  HAL_ERROR;
    }

    /* Check if LIS2MDL connected to Sensor Hub. */
    reg.byte = 0;

    for(int i=0;i<10;i++)
    {
        lis2mdl_device_id_get(&lis2mdl_ctx,&reg.byte);
        if( reg.byte != 0 )break;
        platform_delay(10);;
    }

    if( reg.byte == 0 )
    {
        printf("Can't detect LIS2MDL sensor!\n");
        return  HAL_ERROR;
    }

    ret = 0;

    /* Configure LIS2MDL. */
    ret += lis2mdl_block_data_update_set(&lis2mdl_ctx,PROPERTY_ENABLE);
    ret += lis2mdl_offset_temp_comp_set(&lis2mdl_ctx,PROPERTY_ENABLE);
    ret += lis2mdl_operating_mode_set(&lis2mdl_ctx,LIS2MDL_CONTINUOUS_MODE);
    ret += lis2mdl_data_rate_set(&lis2mdl_ctx,LIS2MDL_ODR_100Hz);

    if( ret != 0 )
    {
        printf("Can't configure LIS2MDL!\n");
        return  HAL_ERROR;
    }

    /*
     * Prepare sensor hub to read data from external Slave0 continuously
     * in order to store data in FIFO.
     */
    sh_cfg_read.slv_add = (LIS2MDL_I2C_ADD & 0xFEU)>>1; /* 7bit I2C address */
    sh_cfg_read.slv_subadd = LIS2MDL_OUTX_L_REG;
    sh_cfg_read.slv_len = 6;

    if( lsm6dsox_sh_slv0_cfg_read(&lsm6dsox_ctx,&sh_cfg_read) != 0 )
    {
        printf("Can't prepare sensor hub!\n");
        return  HAL_ERROR;
    }

    /* Configure Sensor Hub to read one slave. */
    if( lsm6dsox_sh_slave_connected_set(&lsm6dsox_ctx,LSM6DSOX_SLV_0) != 0 )
    {
        printf("Can't configure Sensor Hub to read one slave!\n");
        return  HAL_ERROR;
    }

    /* Enable I2C Master. */
    if( lsm6dsox_sh_master_set(&lsm6dsox_ctx,PROPERTY_ENABLE) != 0 )
    {
        printf("Can't enable I2C Master!\n");
        return  HAL_ERROR;
    }

    /* Configure LSM6DSOX. */
    ret += lsm6dsox_xl_full_scale_set(&lsm6dsox_ctx,LSM6DSOX_8g);
    ret += lsm6dsox_gy_full_scale_set(&lsm6dsox_ctx,LSM6DSOX_2000dps);
    ret += lsm6dsox_block_data_update_set(&lsm6dsox_ctx,PROPERTY_ENABLE);
    ret += lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx,LSM6DSOX_XL_ODR_104Hz);
    ret += lsm6dsox_gy_data_rate_set(&lsm6dsox_ctx,LSM6DSOX_GY_ODR_104Hz);

    if( ret != 0 )
    {
        printf("Can't configure LSM6DSOX!\n");
        return  HAL_ERROR;
    }

    return  HAL_OK;
}

void	config_imu_interface(void)
{
    lsm6dsox_ctx.handle = NULL;
    lsm6dsox_ctx.read_reg = LSM6DSOX_register_read;
    lsm6dsox_ctx.write_reg = LSM6DSOX_register_write;

    lis2mdl_ctx.handle = NULL;
    lis2mdl_ctx.read_reg = LIS2MDL_register_read;
    lis2mdl_ctx.write_reg = LIS2MDL_register_write;

    initialize_imu_device();
}

int32_t	lsm6dsox_sh_lis2mdl(void)
{
    axis3bit16_t    raw_data;
    lsm6dsox_reg_t  reg;
    int32_t			ret;

    do{
        if( (ret=lsm6dsox_status_reg_get(&lsm6dsox_ctx, &reg.status_reg)) != HAL_OK )
        {
        	return	ret;
        }

        if (reg.status_reg.xlda && reg.status_reg.gda)
        {
            /* Read XL data. */
            if( (ret=lsm6dsox_acceleration_raw_get(&lsm6dsox_ctx, raw_data.i16bit)) != HAL_OK )
            {
            	printf("\n");
            	return	ret;
            }
            printf("%d %d %d ",raw_data.i16bit[0],raw_data.i16bit[1],raw_data.i16bit[2]);

            /* Read GY data. */
            if( (ret=lsm6dsox_angular_rate_raw_get(&lsm6dsox_ctx, raw_data.i16bit)) != HAL_OK )
            {
            	printf("\n");
            	return	ret;
            }
            printf("%d %d %d ",raw_data.i16bit[0],raw_data.i16bit[1],raw_data.i16bit[2]);

            /* Read MAG data. */
            if( (ret=lsm6dsox_sh_read_data_raw_get(&lsm6dsox_ctx,
              (lsm6dsox_emb_sh_read_t*) raw_data.u8bit, sizeof(raw_data))) != HAL_OK )
            {
            	printf("\n");
            	return	ret;
            }
            printf("%d %d %d",raw_data.i16bit[0],raw_data.i16bit[1],raw_data.i16bit[2]);
        }
    }while(0);

    printf("\n");
    return  HAL_OK;
}
/* USER CODE END 1 */
