/*
 * dsp_filter.c
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */
#include <math.h>
#include "dsp_filter.h"
#include "dsp_utils.h"
#include "dsp_enum.h"
#include "dsp_config.h"
#include <stdint.h>

void init_filter_state(filter_state_t *state){
	state->s1_l = 0.0f;
	state->s1_r = 0.0f;
	state->s2_l = 0.0f;
	state->s2_r = 0.0f;
}
void set_filter_range(filter_par_t *par, float range){
	par->freq_range = range;
}
void state_variable_filter(float *in_buf, float *out_buf, int len, filter_par_t *par, filter_state_t *state) {
	float freq_lfo_val = 0.0f;
    if (par->enb == ENABLE_SW) {
		if (par->freq_lfo_sel == LFO2){
			freq_lfo_val = *par->val_lfo2;
		}
		else{
			freq_lfo_val = *par->val_lfo1;
		}
		if (par->freq_range < 100.0f) par->freq_range = 100.0f;
        float freq = par->f_0 + freq_lfo_val * par->freq_range * par->lfo_depth_freq;

        if (freq > par->freq_range) freq = par->freq_range;
        if (freq < 10.0f) freq = 10.0f;

        double g = tan((double)M_PI_F * (double)freq / (double)SAMPLE_RATE_SYNTH);
        double k = 1.0 / par->q;

        double a1 = 1.0 / (1.0 + g * (g + k));

        for (uint16_t i = 0; i < len; i += 2) {
            double x_l = (double)in_buf[i];
            double x_r = (double)in_buf[i+1];

            double hp_l = (x_l - (g + k) * state->s1_l - state->s2_l) * a1;
            double bp_l = g * hp_l + state->s1_l;
            double lp_l = g * bp_l + state->s2_l;

            double hp_r = (x_r - (g + k) * state->s1_r - state->s2_r) * a1;
			double bp_r = g * hp_r + state->s1_r;
			double lp_r = g * bp_r + state->s2_r;

            state->s1_l = g * hp_l + bp_l;
            state->s2_l = g * bp_l + lp_l;

            state->s1_r = g * hp_r + bp_r;
			state->s2_r = g * bp_r + lp_r;

            double value_l = 0.0;
            double value_r = 0.0;
			if(par->lerp_val_mode <= 0.5){
				value_l = lp_l;
				value_r = lp_r;
			}
			else if(par->lerp_val_mode <= 1.5){
				value_l = lerp(lp_l, bp_l, par->lerp_val_mode - 0.5);
				value_r = lerp(lp_r, bp_r, par->lerp_val_mode - 0.5);
			}
			else if(par->lerp_val_mode <= 2.0){
				value_l = bp_l;
				value_r = bp_r;
			}
			else if(par->lerp_val_mode <= 3.0){
				value_l = lerp(bp_l, hp_l, par->lerp_val_mode - 2.0);
				value_r = lerp(bp_r, hp_r, par->lerp_val_mode - 2.0);
			}
			else if(par->lerp_val_mode <= 3.5){
				value_l = hp_l;
				value_r = hp_r;
			}
			out_buf[i] = (float)value_l;
			out_buf[i+1] = (float)value_r;
        }
    }
}


