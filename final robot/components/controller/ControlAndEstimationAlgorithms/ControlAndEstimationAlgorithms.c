#include "ControlAndEstimationAlgorithms.h"
#include "GimbalControl.h"

float BR_chassis_LQR_K[4];												// K gain of LQR for the Balancing Robot
float BR_gimbal_LQR_K[2][4];
float Tcc = 0.001;							//inverse of the frequency of chassis controller (1000 Hz)
float Tgc = 0.001;

extern gimbal_model_t standard_gimbal;


void LQR_gain_init(int id_robot_part) {
	
	if (id_robot_part == BR_CHASSIS) {
		
		BR_chassis_LQR_K[0] = 0;
		BR_chassis_LQR_K[1] = -0.3533;
		BR_chassis_LQR_K[2] = 2.7285;
		BR_chassis_LQR_K[3] = 0.7700;
	}
	else if (id_robot_part == STANDARD_GIMBAL) {
		
		BR_gimbal_LQR_K[0][0] = 7.0711;
		BR_gimbal_LQR_K[0][1] = 0;
		BR_gimbal_LQR_K[0][2] = 5.5246;
		BR_gimbal_LQR_K[0][3] = 0;
		BR_gimbal_LQR_K[1][0] = 0;
		BR_gimbal_LQR_K[1][1] = 7.0711;
		BR_gimbal_LQR_K[1][2] = 0;
		BR_gimbal_LQR_K[1][3] = 5.5334;
	}
	else {
		return;
	}
}


void LQR_controller(int id_robot_part) {
	
	if (id_robot_part == BR_CHASSIS) {
		
		// ...
	}
	else if (id_robot_part == STANDARD_GIMBAL) {
		
		float cmd1 = 0;
		float cmd2 = 0;
		for (int i = 0; i < 4; i++) {
			cmd1 += BR_gimbal_LQR_K[0][i]*(standard_gimbal.ref[i] - standard_gimbal.state_estim[i]);
			cmd2 += BR_gimbal_LQR_K[1][i]*(standard_gimbal.ref[i] - standard_gimbal.state_estim[i]);
		}
		standard_gimbal.control_signals[0] = (int16_t) gain_LQR_control_signal_standard_gimbal_1*cmd1;
		standard_gimbal.control_signals[1] = (int16_t) gain_LQR_control_signal_standard_gimbal_2*cmd2;
		
	}
	else {
		return;
	}
}


void sliding_mode_controller(int id_robot_part) {
	
	if (id_robot_part == BR_CHASSIS) {
		
		// ...
	}
	else if (id_robot_part == STANDARD_GIMBAL) {
		
		// ...
	}
	else {
		return;
	}
	
}




/***************************************************************************************
*Name     	: kalman_filter_nonlinear_update
*Function 	: estimates the robot's state (linear position/velocity on the ground and angular position/velocity of the robot) by its inputs and outputs, using a nonlinear model of the balancing robot
*Input    	: u (input to the robot at time k-1), y1, y2, y3, y4 (robot's measured state at time k), y1_estim, y2_estim, y3_estim, y4_estim (pointers to the variables containing the estimations of the robot's state at time k)
*Output   	: none (the function just updates the estimations of robot's state)
*Description: it implements a state-of-the-art algorithm to solve iteratively the Riccatti equation, hence to compute Kf (the Kalman Filter gain matrix) and to estimate the robot's state
****************************************************************************************/

void kalman_filter_nonlinear(int id_robot_part, int id_wheel, int nx, int nu, int ny)
{
	int i, j, k;
	
	if (id_robot_part == BR_CHASSIS) {			// balancing robot (either left wheel or right wheel)
		
			// ...
	}
	else if (id_robot_part == STANDARD_GIMBAL) {
		
		// ...
	}
	else {																			// no robot has been chosen
		return;
	}
	
}