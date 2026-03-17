/*
 * dsp_lfo.c
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */
#include "dsp_utils.h"
#include "dsp_lfo.h"
#include "dsp_enum.h"
#include "dsp_config.h"
#include <math.h>

void init_lfo_state(lfo_state_t *state){
	state->phase_lfo1 = 0.0;
	state->phase_lfo2 = 0.0;
}
float get_lfo_value(float lerp_val, float freq, double *t, float len) {
	if (freq < 0.5f) freq = 1.0f;
	if (freq > 20.0f) freq = 20.0f;
    double phase_inc = freq * len / 2 / SAMPLE_RATE_SYNTH;

    if (lerp_val < 0.0f) lerp_val = 0.0f;
    if (lerp_val > 4.0f) lerp_val = 4.0f;

    float value = 0.0f;
	if(lerp_val <= 1.0f){
		// Sin to Triangle Wave
		float value_sin = sinf(2 * M_PI_F * *t);
		float value_tri = 1.0f- 4.0f* fabsf(*t - 0.5f);
		value = lerp(value_sin, value_tri, lerp_val);
	}
	else if(lerp_val <= 2.0f){
		// Triangle to Ramp Down
		float value_tri = 1.0f- 4.0f* fabsf(*t - 0.5f);
		float value_ramp_down = 2.0f* *t - 1.0;
		value = lerp(value_tri, value_ramp_down, lerp_val - 1.0f);
	}
	else if(lerp_val <= 3.0){
		// Ramp down to Ramp up
		float value_ramp_down = 2.0f* *t - 1.0f;
		float value_ramp_up = -2.0f* *t + 1.0f;
		value = lerp(value_ramp_down, value_ramp_up, lerp_val - 2.0f);
	}
	else if(lerp_val <= 4.0f){
		// Ramp up to Square
		float value_ramp_up = -2.0f * *t + 1.0f;
		float value_sqr = *t < 0.5f ? 1.0f : -1.0f;
		value = lerp(value_ramp_up, value_sqr, lerp_val - 3.0f);
	}
	*t += phase_inc;
	if (*t >= 1.0f) *t -= 1.0f;
	return value;
}

void full_lfo_block(lfo_par_t* par, lfo_state_t *state, int len) {
	float lerpval_lfo2_temp = par->lerp_val_lfo2;
	float freq_lfo2_temp = par->freq_lfo2;
	par->val_lfo1 = get_lfo_value(par->lerp_val_lfo1, par->freq_lfo1, &state->phase_lfo1, len);
	if(par->lfo2_slave_mode == ENABLE_SW){
		if(par->modul_mode == SHAPE){
			lerpval_lfo2_temp += par->val_lfo1 * par->mod_depth * 4.0f;
		}
		else
			freq_lfo2_temp += par->val_lfo1 * par->mod_depth * 20.0f;
	}
	par->val_lfo2 = get_lfo_value(lerpval_lfo2_temp, freq_lfo2_temp, &state->phase_lfo2, len);
}
