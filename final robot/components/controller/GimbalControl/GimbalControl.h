#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdint.h>

/* balancing robot parameters */
typedef struct gimbal_model_s {
	
	// static scalar parameters
	float Iz1;						// inertia of bottom motor of the gimbal along z axis (kg*m^2)
	float Ix2;						// inertia of top motor of the gimbal along x axis (kg*m^2)
	float Iy2;						// inertia of top motor of the gimbal along y axis (kg*m^2)
	float Iz2;						// inertia of top motor of the gimbal along z axis (kg*m^2)
	float M1;							// mass
	float M2;
	float R1;							// radius
	float R2;
	float L1;							// length
	float L2;
	float fv1;						// friction coefficient in the joint of the bottom motor
	float fv2;						// friction coefficient in the joint of the top motor
	
	// static matrices
	float A[4][4];				/* entries of the discrete dynamics matrix (A) */
	float B[4][2];				/* entries of the discrete inputs matrix (B) */
	float C[4][4];				/* entries of the outputs matrix (C) */
	float Vd[4][4];				/* entries of the disturbance covariance matrix (Vd) */
	float Vn[4][4];				/* entries of the noise covariance matrix (Vn) */
	
	// matrices that change at every KF iteration, but we don't need to keep track of their values for next iterations
	float Kf[4][4];				/* initial entries of the Kalman filter matrix (Kf) */
	float tmp[4][4];			/* entries of 'tmp' matrix (matrix that temporarily stores values waiting to be used for other operations) */
	float state_pred[4];	/* predictions of the robot's state */
	float P_pred[4][4];		/* entries of the predicted error covariance matrix of 'x - x_estim' (P_pred) */
	float APA[4][4];			/* entries of the matrix A*P*A^(T) (at every iteration of the Kalman Filter it gets computed from scratch) */
	float CPC[4][4];			/* entries of the matrix C*P_prec*C^(T) (at every iteration of the Kalman Filter it gets computed from scratch) */
	float S[4][4];				/* entries of the innovation covariance matrix (S) */
	float S_array_format[16];
	float S_inv[4][4];		/* entries of the inversed innovation covariance matrix (S^(-1)) */
	float S_inv_array_format[16];
	float motors_real_torque[2];	/* torque actually generated by the bottom and top motor (u) */
	float y[4];							/* output measurements array (y) */
	float delta_y[4];				/* entries of the output prediction error array (delta_y) */

	// matrices that change at every KF iteration, and we have to keep track of their values
	float state_estim[4];		/* estimations of the gimbal's state */
	float P[4][4];					/* entries of the error covariance matrix of 'x - x_estim' (P) */
	
	// reference signals
	float ref[4];
	float ref_prev[4];
	
	// control signals
	int16_t control_signals[2];		// control signals to be sent to bottom and top motor of the gimbal
	
	//SMC parameters
	float a_smc_Pitch_1;		//value a(x) used for SMC (Sliding Mode Control) of Pitch and Pitch_dot of bottom motor
	float b_smc_Pitch_1;		//value b(x) used for SMC (Sliding Mode Control) of Pitch and Pitch_dot of bottom motor
	float k_smc_Pitch_1;		// coeffient for the sliding function: S(k) = e_pitch_dot(k) + k_smc_Pitch_1*e_pitch(k)
	float a_smc_Pitch_2;		//value a(x) used for SMC (Sliding Mode Control) of Pitch and Pitch_dot of top motor
	float b_smc_Pitch_2;		//value b(x) used for SMC (Sliding Mode Control) of Pitch and Pitch_dot of top motor
	float k_smc_Pitch_2;		// coeffient for the sliding function: S(k) = e_pitch_dot(k) + k_smc_Pitch_2*e_pitch(k)
	float smc_conv_coeff;		//convergence coefficient of the discrete-time sliding mode controller: |S(k+1)| <= smc_conv_coeff * |S(k)|
	
} gimbal_model_t;


extern gimbal_model_t BR_gimbal;
//extern float *state_estim;

void gimbal_control_loop(void);
void gimbal_control_init(void);
void gimbal_state_pred(gimbal_model_t *model);



#endif


