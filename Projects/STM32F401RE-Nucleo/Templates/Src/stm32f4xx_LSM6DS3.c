/**
 ******************************************************************************
 * @file    LSM6DS3.c
 * @author  MEMS Application Team
 * @version V1.0.0
 * @date    30-July-2014
 * @brief   This file provides a set of functions needed to manage the LSM6DS3.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_LSM6DS3.h"

#include <stdio.h>
#include <math.h>

I2C_HandleTypeDef I2C_LSM6DS3_Handle;
uint32_t I2C_LSM6DS3_Timeout = LSM6DS3_FLAG_TIMEOUT;

static int16_t LSM6DS3_ACCx,LSM6DS3_ACCy,LSM6DS3_ACCz;
static int16_t LSM6DS3_GYROx,LSM6DS3_GYROy,LSM6DS3_GYROz;

static int16_t cLSM6DS3_ACCx =0;
static int16_t cLSM6DS3_ACCy =0;
static int16_t cLSM6DS3_ACCz =0;

static int16_t cLSM6DS3_GYROx =0;
static int16_t cLSM6DS3_GYROy =0;
static int16_t cLSM6DS3_GYROz =0;

void I2C_Master_Read_Single(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	#if 1
	while(HAL_I2C_Master_Receive(&I2C_LSM6DS3_Handle, (uint16_t)DevAddress, (uint8_t *)pData, Size,10000) != HAL_OK)
	{
		/* Error_Handler() function is called when Timeout error occurs.
		When Acknowledge failure occurs (Slave don't acknowledge it's address)
		Master restarts communication */
		if (HAL_I2C_GetError(&I2C_LSM6DS3_Handle) != HAL_I2C_ERROR_AF)
		{
			appLSM6DS3_I2C_DeInit();
			DEBUG(__FILE__, __LINE__);
			return ;
		}
	}
	#endif	

	
	#if 0
	do
	{	
		if(HAL_I2C_Master_Receive_IT(&I2C_LSM6DS3_Handle, DevAddress, (uint8_t *)pData, Size)!= HAL_OK)
		{
			appLSM6DS3_I2C_DeInit();
			DEBUG(__FILE__, __LINE__);
		}
		while (HAL_I2C_GetState(&I2C_LSM6DS3_Handle) != HAL_I2C_STATE_READY)
		{
		}	
	
	}while(HAL_I2C_GetError(&I2C_LSM6DS3_Handle) ==HAL_I2C_ERROR_AF);
	#endif	
}

void I2C_Master_Write_Single(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	#if 1
	while(HAL_I2C_Master_Transmit(&I2C_LSM6DS3_Handle, (uint16_t)DevAddress, (uint8_t*)pData, Size, 10000)!= HAL_OK)
	{
		/* Error_Handler() function is called when Timeout error occurs.
		When Acknowledge failure occurs (Slave don't acknowledge its address)
		Master restarts communication */
		if (HAL_I2C_GetError(&I2C_LSM6DS3_Handle) != HAL_I2C_ERROR_AF)
		{
			appLSM6DS3_I2C_DeInit();
			DEBUG(__FILE__, __LINE__);
			return ;
		}
	}
	#endif

	#if 0
	do
	{
		if(HAL_I2C_Master_Transmit_IT(&I2C_LSM6DS3_Handle, DevAddress,(uint8_t*)pData, Size)!= HAL_OK)
		{
			appLSM6DS3_I2C_DeInit();
			DEBUG(__FILE__, __LINE__);
		}
		while (HAL_I2C_GetState(&I2C_LSM6DS3_Handle) != HAL_I2C_STATE_READY)
		{
		}
		
	}while(HAL_I2C_GetError(&I2C_LSM6DS3_Handle) ==HAL_I2C_ERROR_AF);
	#endif
}


uint8_t appLSM6DS3_Read(uint8_t DeviceAddr, uint8_t RegisterAddr,
                              uint16_t NumByteToRead,
                              uint8_t* pBuffer)
{
	HAL_StatusTypeDef status = HAL_OK;

	#if defined (HAL_I2C_LIBRARY_MEM)
	{
		status = HAL_I2C_Mem_Read(&I2C_LSM6DS3_Handle, DeviceAddr, ( uint16_t )RegisterAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead,
		                         I2C_LSM6DS3_Timeout);
	}
	#endif

	#if defined (HAL_I2C_LIBRARY_DISCRETE)
	{
		uint8_t Cmd = RegisterAddr;
		uint16_t Len = NumByteToRead;

		I2C_Master_Write_Single(DeviceAddr,(uint8_t*)&Cmd,1);//write which function code
		I2C_Master_Write_Single(DeviceAddr,(uint8_t*)&Len,2);//request how many bytes data
		I2C_Master_Read_Single(DeviceAddr,(uint8_t*)pBuffer,NumByteToRead);//get data  , with requested bytes 
	}
	#endif	

	/* Check the communication status */
	if( status != HAL_OK )
	{
		/* Execute user timeout callback */
		appLSM6DS3_I2C_DeInit();
		return 1;
	}
	else
	{
		return 0;
	}
	
}

uint8_t appLSM6DS3_Write(uint8_t DeviceAddr, uint8_t RegisterAddr,
                               uint16_t NumByteToWrite,
                               uint8_t* pBuffer)
{
	HAL_StatusTypeDef status = HAL_OK;

	#if defined (HAL_I2C_LIBRARY_MEM)
	{
		status = HAL_I2C_Mem_Write(&I2C_LSM6DS3_Handle, DeviceAddr, ( uint16_t )RegisterAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToWrite,
		                          I2C_LSM6DS3_Timeout);
	}
	#endif

	#if defined (HAL_I2C_LIBRARY_DISCRETE)
	{
		uint8_t Cmd = RegisterAddr;
		uint16_t Len = NumByteToWrite;

		I2C_Master_Write_Single(DeviceAddr,(uint8_t*)&Cmd,1);//write which function code
		I2C_Master_Write_Single(DeviceAddr,(uint8_t*)&Len,2);//request how many bytes data
		I2C_Master_Write_Single(DeviceAddr,(uint8_t*)pBuffer,NumByteToWrite);//get data  , with requested bytes 
	}
	#endif	
	
	/* Check the communication status */
	if( status != HAL_OK )
	{
		/* Execute user timeout callback */
		appLSM6DS3_I2C_DeInit();
		return 1;
	}
	else
	{
		return 0;
	}
	
}

void appLSM6DS3_SetAccCalData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			cLSM6DS3_ACCx = data; 
			break;

		case AXIS_Y:
			cLSM6DS3_ACCy = data; 
			break;

		case AXIS_Z:
			cLSM6DS3_ACCz = data; 
			break;				
	}
}

void appLSM6DS3_SetGyroCalData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			cLSM6DS3_GYROx = data; 
			break;

		case AXIS_Y:
			cLSM6DS3_GYROy = data; 
			break;

		case AXIS_Z:
			cLSM6DS3_GYROz = data; 
			break;				
	}
}

void appLSM6DS3_SetAccData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			LSM6DS3_ACCx = data; 
			break;

		case AXIS_Y:
			LSM6DS3_ACCy = data; 
			break;

		case AXIS_Z:
			LSM6DS3_ACCz = data; 
			break;				
	}
}

void appLSM6DS3_SetGyroData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			LSM6DS3_GYROx = data; 
			break;

		case AXIS_Y:
			LSM6DS3_GYROy = data; 
			break;

		case AXIS_Z:
			LSM6DS3_GYROz = data; 
			break;				
	}
}


int16_t appLSM6DS3_GetAccData(AXIS_TypeDef axis)
{
	int16_t data = 0;
	
	switch(axis)
	{
		case AXIS_X:
			data = LSM6DS3_ACCx + cLSM6DS3_ACCx; 
			break;

		case AXIS_Y:
			data = LSM6DS3_ACCy + cLSM6DS3_ACCy; 
			break;

		case AXIS_Z:
			data = LSM6DS3_ACCz + cLSM6DS3_ACCz; 
			break;				
	}
	
	return data ;
}

int16_t appLSM6DS3_GetGyroData(AXIS_TypeDef axis)
{
	int16_t data = 0;
	
	switch(axis)
	{
		case AXIS_X:
			data = LSM6DS3_GYROx + cLSM6DS3_GYROx; 
			break;

		case AXIS_Y:
			data = LSM6DS3_GYROy + cLSM6DS3_GYROy; 
			break;

		case AXIS_Z:
			data = LSM6DS3_GYROz + cLSM6DS3_GYROz; 
			break;				
	}
	
	return data ;
}


void appLSM6DS3_GetAcc(void)
{
	uint8_t tmpxl, tmpxh, tmpyl, tmpyh, tmpzl, tmpzh, tmp;
	int16_t ax_s,ay_s,az_s;
	uint8_t u8WaitCnt=0;
    float sensitivity = LSM6DS3_XL_FS_2G_SENSITIVITY;	//default
	
	do{
		appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_STATUS_REG, 1,&tmp);
		if (u8WaitCnt++>30)
			break;
	}while(!(tmp&BIT(0)));

	#if 1	//calculate linear acceleration in mg
	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1,&tmp);
    tmp &= LSM6DS3_XL_FS_MASK;
//	printf("tmp(A) = 0x%2X\r\n",tmp);//debug
    switch(tmp)
    {
      case LSM6DS3_XL_FS_2G:
        sensitivity = LSM6DS3_XL_FS_2G_SENSITIVITY;
        break;
      case LSM6DS3_XL_FS_4G:
        sensitivity = LSM6DS3_XL_FS_4G_SENSITIVITY;
        break;
      case LSM6DS3_XL_FS_8G:
        sensitivity = LSM6DS3_XL_FS_8G_SENSITIVITY;
        break;
      case LSM6DS3_XL_FS_16G:
        sensitivity = LSM6DS3_XL_FS_16G_SENSITIVITY;
        break;
    }
	#endif	

	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_X_H_XL, 1, &tmpxh);
	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_X_L_XL, 1, &tmpxl);
	ax_s=(((int16_t) ((tmpxh << 8) | tmpxl)));
//	printf("ax_s:%d,%d,%d\r\n",ax_s,tmpxh,tmpxl);	//debug

	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_Y_H_XL, 1, &tmpyh);
	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_Y_L_XL, 1, &tmpyl);
	ay_s=(((int16_t) ((tmpyh << 8) | tmpyl)));
//	printf("ay_s:%d,%d,%d\r\n",ay_s,tmpyh,tmpyl);	//debug	

	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_Z_H_XL, 1, &tmpzh);
	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_Z_L_XL, 1, &tmpzl);
	az_s=(((int16_t) ((tmpzh << 8) | tmpzl)));
//	printf("az_s:%d,%d,%d\r\n",az_s,tmpzh,tmpzl);	//debug	

	appLSM6DS3_SetAccData(AXIS_X,(int16_t)(ax_s*sensitivity));	
	appLSM6DS3_SetAccData(AXIS_Y,(int16_t)(ay_s*sensitivity));	
	appLSM6DS3_SetAccData(AXIS_Z,(int16_t)(az_s*sensitivity));	

//	drvLSM6DS3_Delay(5);
}

void appLSM6DS3_GetGyro(void)
{
	uint8_t tmpxl, tmpxh, tmpyl, tmpyh, tmpzl, tmpzh, tmp;
	int16_t gx_s,gy_s,gz_s;
	uint8_t u8WaitCnt=0;
    float sensitivity = LSM6DS3_G_FS_125_SENSITIVITY;	//default
    
	do{
		appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_STATUS_REG, 1,&tmp);
		u8WaitCnt++;
		if (u8WaitCnt>30)
			break;
	}while(!(tmp&BIT(1)));

	#if 1	//calculate angular rate in mdps
	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_CTRL2_G, 1,&tmp);
    tmp &= LSM6DS3_G_FS_MASK;
//	printf("tmp(G) = 0x%2X\r\n",tmp);//debug
    switch(tmp)
    {
      case LSM6DS3_G_FS_125:
        sensitivity = LSM6DS3_G_FS_125_SENSITIVITY;
        break;
      case LSM6DS3_G_FS_245:
        sensitivity = LSM6DS3_G_FS_245_SENSITIVITY;
        break;
      case LSM6DS3_G_FS_500:
        sensitivity = LSM6DS3_G_FS_500_SENSITIVITY;
        break;
      case LSM6DS3_G_FS_1000:
        sensitivity = LSM6DS3_G_FS_1000_SENSITIVITY;
        break;
      case LSM6DS3_G_FS_2000:
        sensitivity = LSM6DS3_G_FS_2000_SENSITIVITY;
        break;
    }
	#endif	

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_X_H_G, 1, &tmpxh);
	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_X_L_G, 1, &tmpxl);
	gx_s=( (((int16_t)(tmpxh << 8)) | ((int16_t)tmpxl)));

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_Y_H_G, 1, &tmpyh);
	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_Y_L_G, 1, &tmpyl);
	gy_s=( (((int16_t)(tmpyh << 8)) | ((int16_t)tmpyl)));	

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_Z_H_G, 1, &tmpzh);
	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_Z_L_G, 1, &tmpzl);
	gz_s=( (((int16_t)(tmpzh << 8)) | ((int16_t)tmpzl)));	

	appLSM6DS3_SetGyroData(AXIS_X,(int16_t)(gx_s*sensitivity)/1000);	
	appLSM6DS3_SetGyroData(AXIS_Y,(int16_t)(gy_s*sensitivity)/1000);	
	appLSM6DS3_SetGyroData(AXIS_Z,(int16_t)(gz_s*sensitivity)/1000);	

//	drvLSM6DS3_Delay(5);
}

void appLSM6DS3_SetACC(void)
{
	uint8_t data;

	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1,&data);
//	drvLSM6DS3_Delay(5);	
	
    /* Output Data Rate selection */
    data &= ~(LSM6DS3_XL_ODR_MASK);
    data |= LSM6DS3_XL_ODR_1K66HZ;

    /* Full scale selection */
    data &= ~(LSM6DS3_XL_FS_MASK);
    data |= LSM6DS3_XL_FS_2G;
	
	appLSM6DS3_Write(ACC_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1,&data);
//	drvLSM6DS3_Delay(5);

	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_CTRL9_XL, 1,&data);
//	drvLSM6DS3_Delay(5);	

    /* Enable X axis selection */
    data &= ~(LSM6DS3_XL_XEN_MASK);
    data |= LSM6DS3_XL_XEN_ENABLE;

    /* Enable Y axis selection */
    data &= ~(LSM6DS3_XL_YEN_MASK);
    data |= LSM6DS3_XL_YEN_ENABLE;

    /* Enable Z axis selection */
    data &= ~(LSM6DS3_XL_ZEN_MASK);
    data |= LSM6DS3_XL_ZEN_ENABLE;

	appLSM6DS3_Write(ACC_ADDRESS, LSM6DS3_XG_CTRL9_XL, 1,&data);
//	drvLSM6DS3_Delay(5);	
	
}

void appLSM6DS3_SetGyro(void)
{
	uint8_t data;

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_WHO_AM_I_ADDR, 1, &data);
//	drvLSM6DS3_Delay(5);

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_CTRL2_G, 1,&data);
//	drvLSM6DS3_Delay(5);	
	
    /* Output Data Rate selection */
    data &= ~(LSM6DS3_G_ODR_MASK);
    data |= LSM6DS3_G_ODR_1K66HZ;

    /* Full scale selection */
    data &= ~(LSM6DS3_G_FS_MASK);
    data |= LSM6DS3_G_FS_245;
	
	appLSM6DS3_Write(GYRO_ADDRESS, LSM6DS3_XG_CTRL2_G, 1,&data);
//	drvLSM6DS3_Delay(5);

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_CTRL10_C, 1,&data);
//	drvLSM6DS3_Delay(5);	

    /* Enable X axis selection */
    data &= ~(LSM6DS3_G_XEN_MASK);
    data |= LSM6DS3_G_XEN_ENABLE;

    /* Enable Y axis selection */
    data &= ~(LSM6DS3_G_YEN_MASK);
    data |= LSM6DS3_G_YEN_ENABLE;

    /* Enable Z axis selection */
    data &= ~(LSM6DS3_G_ZEN_MASK);
    data |= LSM6DS3_G_ZEN_ENABLE;

	appLSM6DS3_Write(GYRO_ADDRESS, LSM6DS3_XG_CTRL10_C, 1,&data);
//	drvLSM6DS3_Delay(5);	
	
}

void appLSM6DS3_CommonInit(void)
{
	uint8_t data;


	appLSM6DS3_Read(LSM6DS3_ADDRESS_HIGH, LSM6DS3_XG_CTRL3_C, 1,&data);
//	drvLSM6DS3_Delay(5);	

    /* Enable register address automatically incremented during a multiple byte
       access with a serial interface (I2C or SPI) */
    data &= ~(LSM6DS3_XG_IF_INC_MASK);
    data |= LSM6DS3_XG_IF_INC;
	
	appLSM6DS3_Write(LSM6DS3_ADDRESS_HIGH, LSM6DS3_XG_CTRL3_C, 1,&data);
//	drvLSM6DS3_Delay(5);

	appLSM6DS3_Read(LSM6DS3_ADDRESS_HIGH, LSM6DS3_XG_FIFO_CTRL5, 1,&data);
//	drvLSM6DS3_Delay(5);	

    /* FIFO ODR selection */
    data &= ~(LSM6DS3_XG_FIFO_ODR_MASK);
    data |= LSM6DS3_XG_FIFO_ODR_NA;

    /* FIFO mode selection */
    data &= ~(LSM6DS3_XG_FIFO_MODE_MASK);
    data |= LSM6DS3_XG_FIFO_MODE_BYPASS;

	appLSM6DS3_Write(LSM6DS3_ADDRESS_HIGH, LSM6DS3_XG_FIFO_CTRL5, 1,&data);
//	drvLSM6DS3_Delay(5);	
	
}

void appLSM6DS3_Setup(void)
{
	appLSM6DS3_CommonInit();
	appLSM6DS3_SetGyro();
	appLSM6DS3_SetACC();
}	

void appLSM6DS3_SelfTest(void)
{
	appLSM6DS3_GetAcc();
	appLSM6DS3_GetGyro();

	#if 0	//debug
	printf("ACC:%6d,%6d,%6d, GYRO:%6d,%6d,%6d\r\n",
				appLSM6DS3_GetAccData(AXIS_X),appLSM6DS3_GetAccData(AXIS_Y),appLSM6DS3_GetAccData(AXIS_Z),
				appLSM6DS3_GetGyroData(AXIS_X),appLSM6DS3_GetGyroData(AXIS_Y),appLSM6DS3_GetGyroData(AXIS_Z));
	#endif
}	


uint8_t appLSM6DS3_I2C_Init(void)
{ 
	if(HAL_I2C_GetState( &I2C_LSM6DS3_Handle) == HAL_I2C_STATE_RESET )
	{
		/* I2C_EXPBD peripheral configuration */
		I2C_LSM6DS3_Handle.Init.ClockSpeed 		= LSM6DS3_I2C_SPEED;
		I2C_LSM6DS3_Handle.Init.DutyCycle 		= I2C_DUTYCYCLE_2;
		I2C_LSM6DS3_Handle.Init.OwnAddress1    	= 0x33;
		I2C_LSM6DS3_Handle.Init.AddressingMode 	= I2C_ADDRESSINGMODE_7BIT;
		I2C_LSM6DS3_Handle.Instance            	= I2Cx;

		/* Init the I2C */
		HAL_I2C_Init( &I2C_LSM6DS3_Handle );
	}

	if( HAL_I2C_GetState( &I2C_LSM6DS3_Handle) == HAL_I2C_STATE_READY )
	{
		return 0;
	}
	else
	{
		return 1;
	}
  
}

void appLSM6DS3_I2C_DeInit(void)
{
	/* De-initialize the I2C comunication bus */
	HAL_I2C_DeInit( &I2C_LSM6DS3_Handle );

	/* Re-Initiaize the I2C comunication bus */
	appLSM6DS3_I2C_Init();	
}	

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	printf("%s\r\n",__FUNCTION__); 
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	printf("%s\r\n",__FUNCTION__);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("%s\r\n",__FUNCTION__); 
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("%s\r\n",__FUNCTION__); 
}

 void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
	printf("%s\r\n",__FUNCTION__);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
