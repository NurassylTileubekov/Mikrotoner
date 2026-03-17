/*
 * dsp_osc.c
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */
#include "dsp_utils.h"
#include "dsp_osc.h"
#include "dsp_enum.h"
#include "dsp_config.h"
#include <math.h>
#include <stdint.h>

void init_osc_state(osc_state_t *state){
	state->phase_osc1 = 0.0;
	state->phase_osc2 = 0.0;
}
float get_wave_value(float t, float lerp_val, float shape, float offset, double phase_inc) {
	t += offset;
	if (t >= 1.0f) {
		t -= 1.0f;
	}
    if (lerp_val < 0.0f) lerp_val = 0.0f;
    if (lerp_val > 3.0f) lerp_val = 3.0f;

    if (shape < 1.0f) shape = 1.0f;
	if (shape > 4.0f) shape = 4.0f;

    float value = 0.0f;
    float ts = 0.0f;
    float clamp_point = 1.0f / shape;
    uint8_t clamp_wave = 0;
    if(t * shape < 1.0f){
    	ts = t * shape;
    	clamp_wave = 0;
    }
    else{
    	ts = 1.0f;
    	clamp_wave = 1;
    }
	if(lerp_val <= 1.0f){
		// Sin to Triangle Wave
		float value_sin = sinf(2 * M_PI_F * ts);
		float value_tri = 1.0f - 4.0f * fabsf(ts - 0.5f);
		if(clamp_wave){
			value_sin = 0.0f;
			value_tri = -1.0f;
		}
		value = lerp(value_sin, value_tri, lerp_val);
	}
	else if(lerp_val <= 2.0f){
		// Triangle to Sawtooth
		float value_tri = 1.0f - 4.0f * fabsf(ts - 0.5f);
		float value_saw = 2.0f * ts - 1.0f;
		if(clamp_wave){
			value_tri = -1.0f;
			value_saw = -1.0f;
		}
		value_saw -= poly_blep(fmod(t + (1.0 - clamp_point), 1.0), phase_inc);
		value = lerp(value_tri, value_saw, lerp_val - 1.0);
	}
	else if(lerp_val <= 3.0){
		float value_saw = 2.0f * ts - 1.0f;
		float value_sqr = ts < 0.5f ? 1.0f : -1.0f;
		if(clamp_wave){

			value_saw = -1.0f;
			value_sqr = -1.0f;
		}
		value_saw -= poly_blep(fmod(t + (1.0 - clamp_point), 1.0), phase_inc);
		float mid_point = 0.5f / shape;
		value_sqr += poly_blep(t, phase_inc);
		value_sqr -= poly_blep(fmod(t + (1.0 - mid_point), 1.0), phase_inc);
		value = lerp(value_saw, value_sqr, lerp_val - 2.0f);
	}

    return value;
}

void fill_wave_block(float *buf, int len, osc_par_t *par, osc_state_t *state) {
	float lerp_lfo_val_osc1 = 0.0f;
	float lerp_lfo_val_osc2 = 0.0f;
	float pitch_lfo_val_osc1 = 0.0f;
	float pitch_lfo_val_osc2 = 0.0f;
	if(par->lerp_lfo_sel_osc1 == LFO2){
		lerp_lfo_val_osc1 = *par->val_lfo2;
	}
	else{
		lerp_lfo_val_osc1 = *par->val_lfo1;
	}
	if(par->lerp_lfo_sel_osc2 == LFO2){
		lerp_lfo_val_osc2 = *par->val_lfo2;
	}
	else{
		lerp_lfo_val_osc2 = *par->val_lfo1;
	}
	if(par->pitch_lfo_sel_osc1 == LFO2){
		pitch_lfo_val_osc1 = *par->val_lfo2;
	}
	else{
		pitch_lfo_val_osc1 = *par->val_lfo1;
	}
	if(par->pitch_lfo_sel_osc2 == LFO2){
		pitch_lfo_val_osc2 = *par->val_lfo2;
	}
	else{
		pitch_lfo_val_osc2 = *par->val_lfo1;
	}
    float freq_osc2 = 110.0f * powf(2.0f, (par->note + par->fine_osc2 + par->coarse_osc2 + pitch_lfo_val_osc2 * par->lfo_depth_pitch_osc2) / 12.0f);
    float freq_osc1 = 110.0f * powf(2.0f, (par->note + par->fine_osc1 + par->coarse_osc1 + pitch_lfo_val_osc1 * par->lfo_depth_pitch_osc1) / 12.0f);
    double phase_inc_osc2 = freq_osc2 / SAMPLE_RATE_SYNTH;
    double phase_inc_osc1 = freq_osc1 / SAMPLE_RATE_SYNTH;
    float lerp_val_osc1_temp = par->lerp_val_osc1;
    float lerp_val_osc2_temp = par->lerp_val_osc2;
    lerp_val_osc1_temp += lerp_lfo_val_osc1 * par->lfo_lerp_depth_osc1;
	lerp_val_osc2_temp += lerp_lfo_val_osc2 * par->lfo_lerp_depth_osc2;

	float g1_l = cosf(par->pan_osc1 * M_PI_2F);
	float g1_r = sinf(par->pan_osc1 * M_PI_2F);
	float g2_l = cosf(par->pan_osc2 * M_PI_2F);
	float g2_r = sinf(par->pan_osc2 * M_PI_2F);
    for (int i = 0; i < len; i = i + 2) {
    	float value = 0;
    	float value_osc1 = 0;
    	float value_osc2 = 0;
    	if(par->modul_enb == ENABLE_SW){
    		if(par->modul_mod == PHASE){
				//PM modulation
				if (par->vol_osc1 > 0.05f) {
					if (par->vol_osc2 > 0.05f) {
						value_osc2 = get_wave_value(state->phase_osc2, lerp_val_osc2_temp, par->shape_osc2, par->phase_offset_osc2, phase_inc_osc2) *par->vol_osc2;
						float modulated_phase = state->phase_osc1 + par->mod_depth * value_osc2;
						value = get_wave_value(modulated_phase, lerp_val_osc1_temp, par->shape_osc1, par->phase_offset_osc1, phase_inc_osc1) *par->vol_osc1;
					}
					else {
						value = get_wave_value(state->phase_osc1, lerp_val_osc1_temp, par->shape_osc1, par->phase_offset_osc1, phase_inc_osc1) *par->vol_osc1;
					}
				}
				else {
					value = 0.0f;
				}
    		}
    		else if(par->modul_mod == AMPLITUDE){
				//AM modulation
				if(par->vol_osc1 > 0.05f){
					value_osc1 = get_wave_value(state->phase_osc1, lerp_val_osc1_temp, par->shape_osc1, par->phase_offset_osc1, phase_inc_osc1) *par->vol_osc1;
					if(par->vol_osc2 > 0.05f){
					value_osc2 = get_wave_value(state->phase_osc2, lerp_val_osc2_temp, par->shape_osc2, par->phase_offset_osc2, phase_inc_osc2) *par->vol_osc2;
					value = (1.0f + par->mod_depth * value_osc2) * value_osc1;
					}
					else{
						value = value_osc1;
					}
				}
				else{
					value = 0.0f;
				}
    		}
    		buf[i]   = value * g1_l;
			buf[i+1] = value * g1_r;
    	}
    	else{
    		//Addition, no modulation
    		if(par->vol_osc1 > 0.05f){
    			value_osc1 = get_wave_value(state->phase_osc1, lerp_val_osc1_temp, par->shape_osc1, par->phase_offset_osc1, phase_inc_osc1) *par->vol_osc1;
    		}
    		else{
    			value_osc1 = 0.0f;
    		}
    		if(par->vol_osc2 > 0.05f){
    		value_osc2 = get_wave_value(state->phase_osc2, lerp_val_osc2_temp, par->shape_osc2, par->phase_offset_osc2, phase_inc_osc2) *par->vol_osc2;
    		}
    		else{
    			value_osc2 = 0.0f;
    		}
    		value = value_osc1 + value_osc2;
    		buf[i]   = (value_osc1 * g1_l) + (value_osc2 * g2_l);
			buf[i+1] = (value_osc1 * g1_r) + (value_osc2 * g2_r);

    	}
    	state->phase_osc2 += phase_inc_osc2;
    	state->phase_osc1 += phase_inc_osc1;
        if (state->phase_osc1 >= 1.0){
        	state->phase_osc1 -= 1.0;
        	if(par->phase_sync == ENABLE_SW){
        		state->phase_osc2 = 0.0f;
			}
        }
        if (state->phase_osc2 >= 1.0) state->phase_osc2 -= 1.0;
    }
}

