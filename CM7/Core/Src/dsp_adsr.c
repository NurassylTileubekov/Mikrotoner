/*
 * dsp_adsr.c
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */
#include "dsp_utils.h"
#include "dsp_adsr.h"
#include "dsp_enum.h"
#include "dsp_config.h"
#include <stdint.h>

void init_adsr_state(adsr_state_t *adsr_state){
	adsr_state->amplitude = 0.0f;
	adsr_state->value = 0.0f;
	adsr_state->step = 0.0f;
	adsr_state->last_mode = 0;
	adsr_state->mode = 0;
	adsr_state->value_lp = (lp_filter_par_t){0.0f, 0.0f, 0.0f};
}
void adsr_enveloper(adsr_par_t *adsr_par_var, adsr_state_t *adsr_state){
	if(adsr_par_var->a < 0.01) adsr_par_var->a = 0.01f;
	if(adsr_par_var->a > 5.0) adsr_par_var->a = 5.0f;

	if(adsr_par_var->d < 0.01) adsr_par_var->d = 0.01f;
	if(adsr_par_var->d > 5.0) adsr_par_var->d = 5.0f;

	if(adsr_par_var->s < 0.0) adsr_par_var->s = 0.0f;
	if(adsr_par_var->s > 1.0) adsr_par_var->s = 1.0f;

	if(adsr_par_var->r < 0.01) adsr_par_var->r = 0.01f;
	if(adsr_par_var->r > 5.0) adsr_par_var->r = 5.0f;
	float slide = 0.0f;
	float t_coeff = (BUF_SIZE/4.0f)/(SAMPLE_RATE_SYNTH);
	if(adsr_par_var->key_pressed){
		switch(adsr_state->step){
		case 0:
			slide = t_coeff / adsr_par_var->a;
			if(slide > (1.0f - adsr_state->value)){
				slide = 1.0f - adsr_state->value;
			}
			adsr_state->value += slide;
			if(adsr_state->value >= 1.0f){
				adsr_state->value = 1.0f;
				adsr_state->step = 1;
			}
			break;
		case 1:
			slide = (t_coeff / adsr_par_var->d) * (1.0f - adsr_par_var->s);
			if(slide > (adsr_state->value - adsr_par_var->s)){
				slide = adsr_state->value - adsr_par_var->s;
			}
			if(adsr_state->value > adsr_par_var->s){
				adsr_state->value -= slide;
			}
			else{
				adsr_state->value = adsr_par_var->s;
			}
			break;
		}
	}
	else{
		slide = t_coeff / adsr_par_var->r;
		if(slide > adsr_state->value){
			slide = adsr_state->value;
		}
		adsr_state->value -= slide;
		if(adsr_state->value <= 0.0f){
			adsr_state->value = 0.0f;
		}
		adsr_state->step = 0;
	}
	if(adsr_state->mode == ENABLE_SW){
		if(!adsr_state->last_mode){
			adsr_state->value = 0.0f;
			adsr_state->last_mode = 1;
		}
		adsr_state->value_lp.x = adsr_state->value;
		apply_lp_filter(&adsr_state->value_lp);
		adsr_state->amplitude = adsr_state->value_lp.y;
	}
	else{
		if(adsr_state->last_mode){
			adsr_state->value = 0.0f;
			adsr_state->last_mode = 0;
		}
		adsr_state->amplitude = adsr_par_var->key_pressure;
	}
	if(adsr_state->amplitude < 0.0f) adsr_state->amplitude = 0.0f;
	if(adsr_state->amplitude > 1.0f) adsr_state->amplitude = 1.0f;
}


