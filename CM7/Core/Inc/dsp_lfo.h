/*
 * dsp_lfo.h
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */

#ifndef INC_DSP_LFO_H_
#define INC_DSP_LFO_H_
#include "dsp_enum.h"

typedef struct {
    float lerp_val_lfo1;
    float freq_lfo1;
    float lerp_val_lfo2;
    float freq_lfo2;
    enb_sw_t lfo2_slave_mode;
    modulation_mode_lfo_t modul_mode;
    float mod_depth;
    float val_lfo1;
    float val_lfo2;
} lfo_par_t;
typedef struct{
    double phase_lfo1;
    double phase_lfo2;
}lfo_state_t;

void init_lfo_state(lfo_state_t *state);
float get_lfo_value(float lerp_val, float freq, double *t, float len);
void full_lfo_block(lfo_par_t* par, lfo_state_t *state, int len);
#endif /* INC_DSP_LFO_H_ */
