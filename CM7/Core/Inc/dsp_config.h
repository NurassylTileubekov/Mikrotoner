/*
 * dsp_config.h
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */

#ifndef INC_DSP_CONFIG_H_
#define INC_DSP_CONFIG_H_

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#ifndef M_PI_2F
#define M_PI_2F 1.57079632679489661923f
#endif

#define BUF_SIZE    64
#ifndef SAMPLE_RATE_SYNTH
#define SAMPLE_RATE_SYNTH 48000.0f
#endif

#define ADC_12B_RES 4095.0f
#define MUXSIZE_MIKROTONER 16
#define SHARED_PAR_NUM_OSC 6
#define SHARED_PAR_NUM_LFO 2
#define CURRENT_ADC_UPDATE_THRESHOLD 819
#define SOFT_TAKEOVER_THRESHOLD 64


#endif /* INC_DSP_CONFIG_H_ */
