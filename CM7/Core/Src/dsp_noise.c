/*
 * dsp_noise.c
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */
#include "dsp_noise.h"
#include <stdint.h>

float generate_white_noise(uint32_t *state){
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return (int)x / 2147483648.0;
}
void init_noise_state(noise_state_t *noise_state){
	noise_state->y_low_mid = 0.0f;
	noise_state->y_low_side = 0.0f;
	noise_state->y_high_mid = 0.0f;
	noise_state->y_high_side = 0.0f;
	noise_state-> white_mid_prev = 0.0f;
	noise_state->white_side_prev = 0.0f;
	noise_state->state_mid  = 0xACE2026;
	noise_state->state_side = 0x1337BEEF;
}
void fill_noise_block(float *in_buf, float *out_buf, int len, noise_par_t noise_par_var, noise_state_t *noise_state) {
	if (noise_par_var.vol > 1.0f) noise_par_var.vol = 1.0f;
	if (noise_par_var.vol < 0.0f) noise_par_var.vol = 0.0f;
	if (noise_par_var.color > 0.99f) noise_par_var.color = 0.99f;
	if (noise_par_var.color < -0.99f) noise_par_var.color = -0.99f;
	float color_param = noise_par_var.color;
	float width_param = noise_par_var.width;
	float vol_param = noise_par_var.vol;

	for (int i = 0; i < len; i = i + 2) {
		float white_mid = generate_white_noise(&noise_state->state_mid);
		float white_side = generate_white_noise(&noise_state->state_side);
		float colored_mid = white_mid;
		float colored_side = white_side;
		if(color_param >= 0.0f){
			float a = color_param;
			noise_state->y_low_mid = (1.0f - a) * white_mid + a * noise_state->y_low_mid;
			colored_mid = noise_state->y_low_mid;
			noise_state->y_low_side = (1.0f - a) * white_side + a * noise_state->y_low_side;
			colored_side = noise_state->y_low_side;
		}
		else {
			float a = 1.0f + color_param;
			noise_state->y_high_mid = a * noise_state->y_high_mid + a * (white_mid - noise_state->white_mid_prev);
			noise_state->white_mid_prev = white_mid;
			colored_mid = noise_state->y_high_mid;
			noise_state->y_high_side = a * noise_state->y_high_side + a * (white_side - noise_state->white_side_prev);
			noise_state->white_side_prev = white_side;
			colored_side = noise_state->y_high_side;
		}
		float value_l = (colored_mid + (colored_side * width_param)) * vol_param;
		float value_r = (colored_mid - (colored_side * width_param)) * vol_param;
		out_buf[i] = value_l + in_buf[i];
		out_buf[i+1] = value_r + in_buf[i+1];
	}
}

