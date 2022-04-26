#include <stdint.h>
#include "GimbalControl.h"
#include "ControlAndEstimationAlgorithms.h"
#include "ControlUtilities.h"
#include "CAN_receive.h"
#include "bmi088driver.h"


gimbal_model_t standard_gimbal;								// gimbal model of the standard robot

extern fp32 ins_correct_angle[3];							// board angular position measurements
extern bmi088_real_data_t bmi088_real_data;		// board angular velocity and linear acceleration measurements
extern motor_measure_t motor_measures[7];			// motors measurements

int need_to_initialize_gimbal = 1;
float gain_torque_voltage = 1;

// references for angular position of the 2 motors (just to test in debug mode)
float ref_theta_1 = 0;
float ref_theta_2 = 0;


float a = 0;

float b = 0;



void gimbal_control_loop(void) {
	
	if (need_to_initialize_gimbal) {
		
		gimbal_control_init();
		need_to_initialize_gimbal = 0;
		osDelay(3000);					// wait 3 seconds before actually controlling the gimbal (can be better?)
	}
	
	// take board measurements
	standard_gimbal.y[0] = (3.1415/180)*ins_correct_angle[1];			// angular position of motor 0x205 (Yaw of gimbal)
	standard_gimbal.y[1] = (3.1415/180)*ins_correct_angle[2];			// angular position of motor 0x206 (Pitch of gimbal)
	standard_gimbal.y[2] = (3.1415/180)*bmi088_real_data.gyro[1];	// angular velocity of motor 0x205
	standard_gimbal.y[3] = (3.1415/180)*bmi088_real_data.gyro[2];	// angular velocity of motor 0x206
	
	// set the reference signals
	standard_gimbal.ref[0] = ref_theta_1;
	standard_gimbal.ref[1] = ref_theta_2;
	standard_gimbal.ref[2] = 0;
	standard_gimbal.ref[3] = 0;
	
	// compute the state estimations			(will have to be done by the Kalman Filter)
	standard_gimbal.state_estim[0] = standard_gimbal.y[0];
	standard_gimbal.state_estim[1] = standard_gimbal.y[1];
	standard_gimbal.state_estim[2] = standard_gimbal.y[2];
	standard_gimbal.state_estim[3] = standard_gimbal.y[3];
	
	// compute the control signals
	LQR_controller(STANDARD_GIMBAL);
	
	// saturation of the control signals (in order to avoid possible damages)
	standard_gimbal.control_signals[0] = legalize_control_signal(standard_gimbal.control_signals[0], 3000);
	standard_gimbal.control_signals[1] = legalize_control_signal(standard_gimbal.control_signals[1], 3000);
	
	// from torque to voltage
	standard_gimbal.control_signals[0] *= gain_torque_voltage;
	standard_gimbal.control_signals[1] *= gain_torque_voltage;
	
	a = standard_gimbal.control_signals[0];
	b = standard_gimbal.control_signals[1];
	
	// send the control signals
	CAN_cmd_gimbal(standard_gimbal.control_signals[0], standard_gimbal.control_signals[0], (int16_t) 0, (int16_t) 0);
	
}

void gimbal_control_init(void) {
	
	gimbal_model_init(&standard_gimbal);			// initialization of the model for the gimbal
	LQR_gain_init(STANDARD_GIMBAL);						// initialization of the LQR gain for the gimbal
}


