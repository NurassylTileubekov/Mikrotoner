/*
 * dsp_enum.h
 *
 *  Created on: Mar 13, 2026
 *      Author: Nurassyl
 */

#ifndef INC_DSP_ENUM_H_
#define INC_DSP_ENUM_H_

typedef enum {
	DISABLE_SW,
	ENABLE_SW
} enb_sw_t;
typedef enum {
	FALSE = 0,
	TRUE = 1
} bool_t;
typedef enum {
	AMPLITUDE,
	PHASE
} modulation_mode_t;
typedef enum {
	FREQUENCY,
	SHAPE
} modulation_mode_lfo_t;
typedef enum {
	LFO1,
	LFO2
} select_lfo_t;

#endif /* INC_DSP_ENUM_H_ */
