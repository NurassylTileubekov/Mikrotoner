/*
 * dsp_adsr.h
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */

#ifndef INC_DSP_ADSR_H_
#define INC_DSP_ADSR_H_
#include "dsp_enum.h"
#include <stdint.h>
#include "dsp_utils.h"

typedef struct {
    float a;
    float d;
    float s;
    float r;
    uint8_t key_pressed;
    float key_pressure;
} adsr_par_t;

typedef struct{
    float amplitude;
	float value;
	uint8_t step;
	uint8_t last_mode;
    enb_sw_t mode;
    lp_filter_par_t value_lp;
}adsr_state_t;

void init_adsr_state(adsr_state_t *adsr_state);
void adsr_enveloper(adsr_par_t *adsr_par_var, adsr_state_t *adsr_state);

#endif /* INC_DSP_ADSR_H_ */
