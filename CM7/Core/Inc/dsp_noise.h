/*
 * dsp_noise.h
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */

#ifndef INC_DSP_NOISE_H_
#define INC_DSP_NOISE_H_
#include "dsp_enum.h"
#include <stdint.h>

typedef struct {
    float vol;
    float color;
    float width;
} noise_par_t;

typedef struct {
    float y_low_mid;
    float y_low_side;
    float y_high_mid;
    float y_high_side;
    float white_mid_prev;
    float white_side_prev;
    uint32_t state_mid;
    uint32_t state_side;
} noise_state_t;

float generate_white_noise(uint32_t *state);
void init_noise_state(noise_state_t *noise_state);
void fill_noise_block(float *in_buf, float *out_buf, int len, noise_par_t noise_par_var, noise_state_t *noise_state);
#endif /* INC_DSP_NOISE_H_ */
