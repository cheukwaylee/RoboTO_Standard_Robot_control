/**
  ****************************ROBOTO TEAM***************************************
  * @file       chassis_task.c/h
  * @brief      task employed to control the chassis of the robot
  *             
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "struct_typedef.h"
#include "INS_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bmi088driver.h"
#include "CAN_receive.h"
#include "GimbalControl.h"



extern fp32 ins_correct_angle[3];
extern bmi088_real_data_t bmi088_real_data;

//int counter_chassis=0;

// what measurement unit do we need to pass? deg or rad?

void chassis_task(void const *pvParameters){
	
//	fp32 INS_angle_control[3] = {INS_angle[0], INS_angle[1], INS_angle[2]};
//	fp32 angular_velocity_control[3] = {bmi088_real_data.gyro[0],bmi088_real_data.gyro[1],bmi088_real_data.gyro[2]};	
	while(1)
	{
		gimbal_control_loop();
		osDelay(1);
	}
}