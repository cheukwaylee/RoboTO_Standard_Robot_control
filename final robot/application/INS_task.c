/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "MahonyAHRS.h"
#include "math.h"
#include "stdio.h"



#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)


/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
static void imu_temp_control(fp32 temp);


void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      
fp32 ins_correct_angle[3] = {0.0f, 0.0f, 0.0f};
fp32 gyro_temperature;
uint16_t counter_gyro;

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};

bool_t bmi088_ist3810_init_flag = 1;  // flag used to perform only once the bmi088 initialization

// char variables useful for UART print, check if 30 is too much
char angle_print_x[30];
char angle_print_y[30];
char angle_print_z[30];
char accel_print_x[30];
char accel_print_y[30];
char accel_print_z[30];
char gyro_temperature_print[30];
char counter_gyro_print[30];


/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void INS_task(void const *pvParameters)
{

	while(1)
	{ 
		if (bmi088_ist3810_init_flag == 1){
		
			while(BMI088_init())											 
			{
        osDelay(100);
			}
		
		while(ist8310_init())											// init of the BMI088 peripheral
			{
        osDelay(100);
			}
			
			// initialization for the PID
			PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
			
			// init for the AHRS algo that will perform the sensor fusion with acc, gyro, mag, temperature and time
			AHRS_init(INS_quat, bmi088_real_data.accel, ist8310_real_data.mag);
			
			bmi088_ist3810_init_flag = 0;
		}
	
		BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);	
		gyro_temperature = bmi088_real_data.temp;
		
//		imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);  

		MahonyAHRSupdateIMU(INS_quat, bmi088_real_data.gyro[0], bmi088_real_data.gyro[1], bmi088_real_data.gyro[2], bmi088_real_data.accel[0], bmi088_real_data.accel[1], bmi088_real_data.accel[2]);					// sensor fusion algo 
		
		// get yaw, pitch, roll angles
		get_angle(INS_quat, &INS_angle[0], &INS_angle[1], &INS_angle[2]);
		
		// convert the values to degrees and correct the numerical drift
		ins_correct_angle[0] = (INS_angle[0] + counter_gyro*0.00000113825)/(0.001454*3); 		// /1.185
		ins_correct_angle[1] = (INS_angle[1] + counter_gyro*0.000002)/(0.001453667*3);			// /1.185
		ins_correct_angle[2] = -(INS_angle[2] - counter_gyro*0.00000003)/(0.001419389*3);			// /1.185
		
//		if (TURN_ON_INS_ANGLE_PRINT == 1)
//		{
//			sprintf(angle_print_x,"\r%f\n\r", ins_correct_angle[0]);  // first number alone works fine if we check 1 axe at a time
//			sprintf(angle_print_y,"%f\n\r", ins_correct_angle[1]);		// first number alone works fine if we check 1 axe at a time
//			sprintf(angle_print_z,"%f\n\r", ins_correct_angle[2]);		// first number alone works fine if we check 1 axe at a time
//			HAL_UART_Transmit(&huart1, (uint8_t *)&angle_print_x, sizeof(angle_print_x), 10);
//			HAL_UART_Transmit(&huart1, (uint8_t *)&angle_print_y, sizeof(angle_print_y), 10);
//			HAL_UART_Transmit(&huart1, (uint8_t *)&angle_print_z, sizeof(angle_print_z), 10);
//		}
		
//		if (TURN_ON_INS_ACCEL_PRINT == 1)
//		{
//			sprintf(accel_print_x,"%f\t", bmi088_real_data.accel[0]);
//			HAL_UART_Transmit(&huart1, (uint8_t *)&accel_print_x, sizeof(accel_print_x), 10);
//			sprintf(accel_print_y,"%f\t", bmi088_real_data.accel[1]);
//			HAL_UART_Transmit(&huart1, (uint8_t *)&accel_print_y, sizeof(accel_print_y), 10);
//			sprintf(accel_print_z,"%f\t", bmi088_real_data.accel[2]);
//			HAL_UART_Transmit(&huart1, (uint8_t *)&accel_print_z, sizeof(accel_print_z), 10);
//		}

//		if (TURN_ON_INS_ANGLE_PRINT == 1 || TURN_ON_INS_ACCEL_PRINT == 1)
//		{
//			sprintf(counter_gyro_print,"%d\t", counter_gyro);
//			HAL_UART_Transmit(&huart1,(uint8_t *) counter_gyro_print , sizeof(counter_gyro_print), 10);
//			
//			sprintf(gyro_temperature_print,"%f\r\n", gyro_temperature);
//			HAL_UART_Transmit(&huart1,(uint8_t *) gyro_temperature_print , sizeof(gyro_temperature_print), 10);
//		}
		
		counter_gyro ++;
		  
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		osDelay(1);
	}
}



static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 45.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])       	// init of the quaternions for AHRS_update (is it correct?? do we have to fill parameters or the 0s are correct??)
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])						// sensor fusion algo
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}

void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)					// gives as output the values that we will use outside the INS_Task using the values obtained from the AHRS_update
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}