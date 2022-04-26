#ifndef CONTROL_UTILITIES_H
#define CONTROL_UTILITIES_H

#include <stdint.h>
#include "GimbalControl.h"

int16_t legalize_control_signal(int16_t signal, int16_t saturation_limit);

void gimbal_model_init(gimbal_model_t *model);

void gimbal_state_pred(gimbal_model_t *model);

#endif