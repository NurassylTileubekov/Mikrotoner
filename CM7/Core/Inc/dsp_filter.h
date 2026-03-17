/*
 * dsp_filter.h
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */

#ifndef INC_DSP_FILTER_H_
#define INC_DSP_FILTER_H_
#include "dsp_enum.h"

typedef struct {
    enb_sw_t enb;
    float f_0;
    float q;
    float lerp_val_mode;
    float freq_range;
    float *val_lfo1;
    float *val_lfo2;
    float lfo_depth_freq;
    select_lfo_t freq_lfo_sel;
} filter_par_t;

typedef struct {
    double s1_l, s2_l;
    double s1_r, s2_r;
} filter_state_t;

void init_filter_state(filter_state_t *state);
void set_filter_range(filter_par_t *par, float range);
void state_variable_filter(float *in_buf, float *out_buf, int len, filter_par_t *f_par, filter_state_t *stt);

#endif /* INC_DSP_FILTER_H_ */
