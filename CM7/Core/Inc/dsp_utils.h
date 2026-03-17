/*
 * dsp_utils.h
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */

#ifndef INC_DSP_UTILS_H_
#define INC_DSP_UTILS_H_

typedef struct {
    float x;
    float y;
    float a;
} lp_filter_par_t;

void init_lp_filter_pars(lp_filter_par_t *p, float freq_cutoff);
void apply_lp_filter(lp_filter_par_t *p);

float soft_clip(float x, float threshold, float gain);

float lerp(float a, float b, float t);

float poly_blep(float t, float dt);

#endif /* INC_DSP_UTILS_H_ */
