#include "ControlUtilities.h"

int16_t legalize_control_signal(int16_t signal, int16_t saturation_limit) {
	
	if (signal > saturation_limit) signal = saturation_limit;
	else if (signal < -saturation_limit) signal = -saturation_limit;
	
	return signal;
}


void gimbal_model_init(gimbal_model_t *model) {
	
	/* physical parameters of the gimbal */
	model->M1 = 2;
	model->M2 = 2.5;
	model->R1 = 0.128;
	model->R2 = 0.200;
	model->L1 = 0.150;
	model->L2 = 0.300;
	model->Iz1 = (1/2)*(model->M1+model->M2)*model->R1*model->R1;
	//model->Ix2 = (1/4)*model->M2*model->R2*model->R2 + (1/12)*model->M2*model->L2*model->L2;
	model->Iy2 = (1/4)*model->M2*model->R2*model->R2 + (1/12)*model->M2*model->L2*model->L2;
	//model->Iz2 = (1/2)*model->M2*model->R2*model->R2;
	//model->fv1 = 0.2;
	//model->fv2 = 0.002;
	
	
	/* entries of the dynamics matrix (A) */
	model->A[0][0] = 1;
	model->A[0][1] = 0;
	model->A[0][2] = 0.0010;
	model->A[0][3] = 0;
	model->A[1][0] = 0;
	model->A[1][1] = 1;
	model->A[1][2] = 0;
	model->A[1][3] = 0.0010;
	model->A[2][0] = 0;
	model->A[2][1] = 0;
	model->A[2][2] = 1;
	model->A[2][3] = 0;
	model->A[3][0] = 0;
	model->A[3][1] = 0;
	model->A[3][2] = 0;
	model->A[3][3] = 1;

	/* entries of the inputs matrix (B) */
	model->B[0][0] = 0;
	model->B[0][1] = 0;
	model->B[1][0] = 0;
	model->B[1][1] = 0;
	model->B[2][0] = 0.0271;
	model->B[2][1] = 0;
	model->B[3][0] = 0;
	model->B[3][1] = 0.0229;
	

	/* entries of the outputs matrix (C) */
	model->C[0][0] = 1;
	model->C[0][1] = 0;
	model->C[0][2] = 0;
	model->C[0][3] = 0;
	model->C[1][0] = 0;
	model->C[1][1] = 1;
	model->C[1][2] = 0;
	model->C[1][3] = 0;
	model->C[2][0] = 0;
	model->C[2][1] = 0;
	model->C[2][2] = 1;
	model->C[2][3] = 0;
	model->C[3][0] = 0;
	model->C[3][1] = 0;
	model->C[3][2] = 0;
	model->C[3][3] = 1;

	/* initial entries of the error covariance matrix of 'x - x_estim' (P) */
	model->P[0][0] = 1;
	model->P[0][1] = 0;
	model->P[0][2] = 0;
	model->P[0][3] = 0;
	model->P[1][0] = 0;
	model->P[1][1] = 1;
	model->P[1][2] = 0;
	model->P[1][3] = 0;
	model->P[2][0] = 0;
	model->P[2][1] = 0;
	model->P[2][2] = 1;
	model->P[2][3] = 0;
	model->P[3][0] = 0;
	model->P[3][1] = 0;
	model->P[3][2] = 0;
	model->P[3][3] = 1;

	/* entries of the disturbance covariance matrix (Vd) */
	model->Vd[0][0] = 0.1;
	model->Vd[0][1] = 0;
	model->Vd[0][2] = 0;
	model->Vd[0][3] = 0;
	model->Vd[1][0] = 0;
	model->Vd[1][1] = 0.1;
	model->Vd[1][2] = 0;
	model->Vd[1][3] = 0;
	model->Vd[2][0] = 0;
	model->Vd[2][1] = 0;
	model->Vd[2][2] = 0.1;
	model->Vd[2][3] = 0;
	model->Vd[3][0] = 0;
	model->Vd[3][1] = 0;
	model->Vd[3][2] = 0;
	model->Vd[3][3] = 0.1;

	/* entries of the noise covariance matrix (Vn) */
  model->Vn[0][0] = 1;
	model->Vn[0][1] = 0;
	model->Vn[0][2] = 0;
	model->Vn[0][3] = 0;
	model->Vn[1][0] = 0;
	model->Vn[1][1] = 1;
	model->Vn[1][2] = 0;
	model->Vn[1][3] = 0;
	model->Vn[2][0] = 0;
	model->Vn[2][1] = 0;
	model->Vn[2][2] = 1;
	model->Vn[2][3] = 0;
	model->Vn[3][0] = 0;
	model->Vn[3][1] = 0;
	model->Vn[3][2] = 0;
	model->Vn[3][3] = 1;
	
	/* state estimations */
	for (int i = 0; i < 4; i++) {
		model->state_estim[i] = 0;
	}
	
	/* reference signals for previous time instant */
	for (int i = 0; i < 4; i++) {
		model->ref_prev[i] = 0;
	}
	
	/* SMC control */
	model->k_smc_Pitch_1 = 4;
	model->k_smc_Pitch_2 = 4;
	model->smc_conv_coeff = 0.1;

}


void gimbal_state_pred(gimbal_model_t *model) {
	
	// ... computation of state_pred for the gimbal
	
}
