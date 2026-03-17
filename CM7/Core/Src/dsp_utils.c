/*
 * dsp_utils.c
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */
#include "dsp_utils.h"
#include "dsp_config.h"
#include <math.h>
void init_lp_filter_pars(lp_filter_par_t *p, float freq_cutoff) {
    p->a = (2.0f * M_PI_F * freq_cutoff) / SAMPLE_RATE_SYNTH;

    if (p->a > 1.0f) p->a = 1.0f;
    if (p->a < 0.0f) p->a = 0.0f;
}
void apply_lp_filter(lp_filter_par_t *p) {
    p->y += p->a * (p->x - p->y);
}

float soft_clip(float x, float threshold, float gain) {
    const float clip_start = threshold * 0.95f;
    const float range = threshold - clip_start;

    if (x > clip_start) {
        float t = (x - clip_start) / range;
        return clip_start + range * tanhf(t * gain);
    } else if (x < -clip_start) {
        float t = (-x - clip_start) / range;
        return -clip_start - range * tanhf(t * gain);
    } else {
        return x;
    }
}

float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

float poly_blep(float t, float dt) {
    if (t < dt) {
        t /= dt;
        return t + t - t * t - 1.0;
    } else if(t > 1.0 - dt) {
        t = (t - 1.0) / dt;
        return t * t + t + t + 1.0;
    }
    return 0.0;
}



