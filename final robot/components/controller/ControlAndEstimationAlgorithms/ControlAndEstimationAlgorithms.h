#ifndef CONTROL_AND_ESTIMATION_ALGORITHMS_H
#define CONTROL_AND_ESTIMATION_ALGORITHMS_H

#define BR_CHASSIS 1
#define BR_GIMBAL 2
#define STANDARD_GIMBAL 3

#define LEFT_WHEEL 1
#define RIGHT_WHEEL 2

#define gain_Pos_dot_from_Pitch 6
#define gain_no_joystick_commands 1.5

#define gain_LQR_control_signal_BR_chassis 1
#define gain_LQR_control_signal_standard_gimbal_1 1
#define gain_LQR_control_signal_standard_gimbal_2 1
#define gain_wheels_from_Pos 600
#define gain_wheels_from_Pitch 60


// variables for control/estimation algorithms
extern float BR_chassis_LQR_K[4];												// K gain of LQR for the Balancing Robot
extern float BR_gimbal_LQR_K[2][4];
extern float Tcc;																				//inverse of the frequency of chassis controller (1000 Hz)
extern float Tgc;


// control algorithms
void LQR_gain_init(int id_robot_part);
void LQR_controller(int id_robot_part);
void sliding_mode_controller(int id_robot_part);

// estimation algorithms
void kalman_filter_nonlinear(int id_robot_part, int id_wheel, int nx, int nu, int ny);




#endif
