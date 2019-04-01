/* Includes ------------------------------------------------------------------*/
#include "MEMS_Calculate.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*angle variable*/
#define PI (float)3.14159265f
float RollAng = 0.0f, PitchAng = 0.0f;

#define FILTER_COUNT  	(16)
#define FILTER_FACTOR  	(4)

#define	GYRO_SCALE 	(1)	//(70)

int16_t ax_buf[FILTER_COUNT], ay_buf[FILTER_COUNT],az_buf[FILTER_COUNT];
int16_t gx_buf[FILTER_COUNT], gy_buf[FILTER_COUNT],gz_buf[FILTER_COUNT];
int16_t gx, gy, gz, ax ,ay, az;
static float angle, angle_dot, f_angle, f_angle_dot;

/*MEMS calibration*/
uint8_t Flag_Calibrate = 0;

/*misc marco*/
#define	MEMSABS(X)			((X) >= 0 ? (X) : -(X))
#define	MEMSCONVERTA(x,y)	(y = (x>=0)?(1):(0))
#define	MEMSCONVERTB(x,y)	(x = (y == 1)?(-x):(x))

extern uint8_t Flag_Button;

/* Private functions ---------------------------------------------------------*/

void Gyroscope_Calibration(void)
{
	int32_t gyroX = 0;
	int32_t gyroY = 0;
	int32_t gyroZ = 0;		

//	uint16_t integerX = 0;
//	uint16_t integerY = 0;	
//	uint16_t integerZ = 0;	
	
	if (Flag_Calibrate)
	{
		gyroX = appLSM6DS3_GetGyroData(AXIS_X);
		if (MEMSABS(gyroX)>0)
		{
			appLSM6DS3_SetGyroCalData(AXIS_X,-gyroX);

		}
		gyroY = appLSM6DS3_GetGyroData(AXIS_Y);
		if (MEMSABS(gyroY)>0)
		{
			appLSM6DS3_SetGyroCalData(AXIS_Y,-gyroY);

		}
		gyroZ = appLSM6DS3_GetGyroData(AXIS_Z);
		if (MEMSABS(gyroZ)>0)
		{
			appLSM6DS3_SetGyroCalData(AXIS_Z,-gyroZ);

		}
		
//		printf("%s : %4d,%4d,%4d\r\n",__FUNCTION__,gyroX,gyroY,gyroZ);
	}
}

void Accelerator_Calibration(void)
{
	int32_t accX = 0;
	int32_t accY = 0;
	int32_t accZ = 0;		

//	uint16_t integerX = 0;
//	uint16_t integerY = 0;	
//	uint16_t integerZ = 0;		
	
	if (Flag_Calibrate)
	{
		appLSM6DS3_SetAccCalData(AXIS_X,0);	//reset calibration data
		accX = appLSM6DS3_GetAccData(AXIS_X);
		if (MEMSABS(accX)>0)
		{
			appLSM6DS3_SetAccCalData(AXIS_X,-accX);

		}

		appLSM6DS3_SetAccCalData(AXIS_Y,0);		//reset calibration data
		accY = appLSM6DS3_GetAccData(AXIS_Y);		
		if (MEMSABS(accY)>0)
		{
			appLSM6DS3_SetAccCalData(AXIS_Y,-accY);

		}

		appLSM6DS3_SetAccCalData(AXIS_Z,0);		//reset calibration data
		accZ = appLSM6DS3_GetAccData(AXIS_Z);		
		if ((MEMSABS(accZ)>1000)||(MEMSABS(accZ)<=999))
		{
			appLSM6DS3_SetAccCalData(AXIS_Z,-accZ+1000);

		}
		
//		printf("%s : %4d,%4d,%4d\r\n",__FUNCTION__,accX,accY,accZ);
	}
}

void Accelerator_filter(void)
{
	#if defined (ENABLE_AVERAGE_FILTER)

	uint8_t i;
	int32_t ax_sum = 0, ay_sum = 0, az_sum = 0; 

	for(i = 1 ; i < FILTER_COUNT; i++)
	{
		ax_buf[i - 1] = ax_buf[i];
		ay_buf[i - 1] = ay_buf[i];
		az_buf[i - 1] = az_buf[i];
	}

	ax_buf[FILTER_COUNT - 1] = appLSM6DS3_GetAccData(AXIS_X);
	ay_buf[FILTER_COUNT - 1] = appLSM6DS3_GetAccData(AXIS_Y);
	az_buf[FILTER_COUNT - 1] = appLSM6DS3_GetAccData(AXIS_Z);

	for(i = 0 ; i < FILTER_COUNT; i++)
	{
		ax_sum += ax_buf[i];
		ay_sum += ay_buf[i];
		az_sum += az_buf[i];
	}

	ax = (int16_t)(ax_sum>>FILTER_FACTOR); //	/ FILTER_COUNT);
	ay = (int16_t)(ay_sum>>FILTER_FACTOR); //	/ FILTER_COUNT);
	az = (int16_t)(az_sum>>FILTER_FACTOR); //	/ FILTER_COUNT);

	#else
	ax = appLSM6DS3_GetAccData(AXIS_X);
	ay = appLSM6DS3_GetAccData(AXIS_Y);
	az = appLSM6DS3_GetAccData(AXIS_Z);

	#endif
	
}

void Gyroscope_filter(void)
{
	#if defined (ENABLE_AVERAGE_FILTER)	
	uint8_t i;
	int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0; 

	for(i = 1 ; i < FILTER_COUNT; i++)
	{
		gx_buf[i - 1] = gx_buf[i];
		gy_buf[i - 1] = gy_buf[i];
		gz_buf[i - 1] = gz_buf[i];
	}

	gx_buf[FILTER_COUNT - 1] = appLSM6DS3_GetGyroData(AXIS_X);
	gy_buf[FILTER_COUNT - 1] = appLSM6DS3_GetGyroData(AXIS_Y);
	gz_buf[FILTER_COUNT - 1] = appLSM6DS3_GetGyroData(AXIS_Z);

	for(i = 0 ; i < FILTER_COUNT; i++)
	{
		gx_sum += gx_buf[i];
		gy_sum += gy_buf[i];
		gz_sum += gz_buf[i];
	}

	gx = (int16_t)(gx_sum>>FILTER_FACTOR);// / FILTER_COUNT);
	gy = (int16_t)(gy_sum>>FILTER_FACTOR);// / FILTER_COUNT);
	gz = (int16_t)(gz_sum>>FILTER_FACTOR);// / FILTER_COUNT);

	#else
	gx = appLSM6DS3_GetGyroData(AXIS_X);
	gy = appLSM6DS3_GetGyroData(AXIS_Y);
	gz = appLSM6DS3_GetGyroData(AXIS_Z);	

	#endif
	
}

void Tilt_Angle_Calculate(void)
{  
	float s1 = 0;
	float s2 = 0;	

	s1 = sqrt((float)((ay *ay )+(az *az )));
	s2 = sqrt((float)((ax *ax )+(az *az )));

	PitchAng = atan(ax /s1)*57.295779;	//*180/PI;
	RollAng = atan(ay /s2)*57.295779;	//*180/PI;

	#if defined (ENABLE_KALMAN_FILTER)
	angle_dot = gx*GYRO_SCALE;	
	kalman_filter(RollAng, angle_dot, &f_angle, &f_angle_dot);
	#endif

	#if 0	//debug

	if (Flag_Button)
	{
		printf("Pitch:%8.3lf,",PitchAng);
		printf("Roll:%8.3lf,",RollAng);
		
		printf("Acc:%5d,%5d,%5d,",ax ,ay ,az );
		printf("Gyro:%5d,%5d,%5d,",gx ,gy ,gz );	
		printf("\r\n");
	}
	else
	{		
		printf("Pitch:%8.3lf,",PitchAng);
		printf("Roll:%8.3lf,",RollAng);

		#if defined (ENABLE_KALMAN_FILTER)
		printf("Angle:%8.3lf,",f_angle);
		#endif
		
		printf("Acc:%5d,%5d,%5d,",ax ,ay ,az );
		printf("Gyro:%5d,%5d,%5d,",gx ,gy ,gz );	
		printf("\r\n");
	}
	
	#endif
}

