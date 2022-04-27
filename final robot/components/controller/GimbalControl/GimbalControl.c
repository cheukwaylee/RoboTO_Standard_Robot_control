#include <stdint.h>
#include "GimbalControl.h"
#include "ControlAndEstimationAlgorithms.h"
#include "ControlUtilities.h"
#include "CAN_receive.h"
#include "bmi088driver.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "CAN_receive.h"

/**************************************************************
Notes about the Gimbal:
- The board is placed on the gimbal in such a way that the axes x,y,z point in directions forward,up,right (respectively)
- The positive rotation for Yaw/Pitch movements is in the left/up directions (according to the 2nd rule of the right hand)
**************************************************************/


gimbal_model_t standard_gimbal;								// gimbal model of the standard robot

int need_to_initialize_gimbal = 1;
float gain_torque_to_voltage = 156/13.33;			// taken from datasheet (156/13.33 = Speed_Torque_Gradient/SpeedConstant)

// references for angular position of the 2 motors (just to test in debug mode)
float ref_gimbal_yaw = 0;
float ref_gimbal_pitch = 0;

// values of the control signals for Yaw and Pitch movements (just to see them in debug mode)
float control_signal_gimbal_yaw_value = 0;
float control_signal_gimbal_pitch_value = 0;

extern fp32 ins_correct_angle[3];
extern bmi088_real_data_t bmi088_real_data;

float MIDDLE_YAW_ANGLE_GIMBAL = 0;											// angle considered as the "center point" for the yaw movement (defined in radiants)
float MIDDLE_PITCH_ANGLE_GIMBAL = 0;										// angle considered as the "center point" for the pitch movement (defined in radiants)
float MAX_GIMBAL_YAW_CONTROL_SIGNAL_AMPLITUDE = 7000;		// defined in range [0,+30000]
float MAX_GIMBAL_PITCH_CONTROL_SIGNAL_AMPLITUDE = 8000;	// defined in range [0,+30000]


void gimbal_control_loop(void) {
	
	if (need_to_initialize_gimbal) {
		
		gimbal_control_init();
		need_to_initialize_gimbal = 0;
	}
	
	// take board measurements
	standard_gimbal.y[0] = (3.1415/180)*ins_correct_angle[1];			// angular position of motor 0x205 (Yaw of gimbal)
	standard_gimbal.y[1] = (3.1415/180)*ins_correct_angle[2];			// angular position of motor 0x206 (Pitch of gimbal)
	standard_gimbal.y[2] = (3.1415/180)*bmi088_real_data.gyro[1];	// angular velocity of motor 0x205
	standard_gimbal.y[3] = (3.1415/180)*bmi088_real_data.gyro[2];	// angular velocity of motor 0x206
	
	// set the reference signals
	standard_gimbal.ref[0] = ref_gimbal_yaw + MIDDLE_YAW_ANGLE_GIMBAL;
	standard_gimbal.ref[1] = ref_gimbal_pitch + MIDDLE_PITCH_ANGLE_GIMBAL;
	standard_gimbal.ref[2] = 0;
	standard_gimbal.ref[3] = 0;
	
	// compute the state estimations			(will have to be done by the Kalman Filter)
	standard_gimbal.state_estim[0] = standard_gimbal.y[0];
	standard_gimbal.state_estim[1] = standard_gimbal.y[1];
	standard_gimbal.state_estim[2] = standard_gimbal.y[2];
	standard_gimbal.state_estim[3] = standard_gimbal.y[3];
	
	// compute the control signals
	LQR_controller(STANDARD_GIMBAL);
	
	// from torque to voltage
	standard_gimbal.control_signals[0] *= gain_torque_to_voltage;
	standard_gimbal.control_signals[1] *= gain_torque_to_voltage;
	
	// from voltage to range [-30000, +30000] (to be sent to the GM6020 motor)
	standard_gimbal.control_signals[0] *= 30000/24;
	standard_gimbal.control_signals[1] *= 30000/24;
	
	// variables defined just to see in debug mode the value of control signals
	control_signal_gimbal_yaw_value = standard_gimbal.control_signals[0];
	control_signal_gimbal_pitch_value = standard_gimbal.control_signals[1];
	
	// saturation of the control signals (in order to avoid possible damages)
	standard_gimbal.control_signals[0] = legalize_control_signal(standard_gimbal.control_signals[0], MAX_GIMBAL_YAW_CONTROL_SIGNAL_AMPLITUDE);
	standard_gimbal.control_signals[1] = legalize_control_signal(standard_gimbal.control_signals[1], MAX_GIMBAL_PITCH_CONTROL_SIGNAL_AMPLITUDE);
	
	// send the control signals
	CAN_cmd_gimbal((int16_t) standard_gimbal.control_signals[0], (int16_t) standard_gimbal.control_signals[1], (int16_t) 0, (int16_t) 0);
	
}

void gimbal_control_init(void) {
	
	gimbal_model_init(&standard_gimbal);			// initialization of the model for the gimbal
	LQR_gain_init(STANDARD_GIMBAL);						// initialization of the LQR gain for the gimbal
}


