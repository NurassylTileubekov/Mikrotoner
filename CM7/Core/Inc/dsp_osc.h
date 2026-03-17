/*
 * dsp_osc.h
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */

#ifndef INC_DSP_OSC_H_
#define INC_DSP_OSC_H_
#include "dsp_enum.h"

typedef struct {
    float note;
    float vol_osc1;
    float vol_osc2;
    float lerp_val_osc2;
    float shape_osc2;
    float coarse_osc2;
    float fine_osc2;
    float phase_offset_osc2;
    float pan_osc2;
    float lerp_val_osc1;
    float shape_osc1;
    float coarse_osc1;
    float fine_osc1;
    float phase_offset_osc1;
    float pan_osc1;
    enb_sw_t modul_enb;
    modulation_mode_t modul_mod;
    float mod_depth;
    enb_sw_t phase_sync;
    float lfo_depth_pitch_osc1;
    select_lfo_t pitch_lfo_sel_osc1;
    float lfo_depth_pitch_osc2;
    select_lfo_t pitch_lfo_sel_osc2;
    float lfo_lerp_depth_osc1;
    select_lfo_t lerp_lfo_sel_osc1;
    float lfo_lerp_depth_osc2;
    select_lfo_t lerp_lfo_sel_osc2;
    float *val_lfo1;
    float *val_lfo2;
} osc_par_t;

typedef struct {
	double phase_osc2;
	double phase_osc1;
}osc_state_t;

void init_osc_state(osc_state_t *state);
float get_wave_value(float t, float lerp_val, float shape, float offset, double phase_inc);
void fill_wave_block(float *buf, int len, osc_par_t *par, osc_state_t *state);

#endif /* INC_DSP_OSC_H_ */
