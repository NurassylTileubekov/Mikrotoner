//AUTHOR: NURASSYL TILEUBEKOV
#define BUF_SIZE    64

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#ifndef M_PI_2F
#define M_PI_2F 1.57079632679489661923f
#endif

#ifndef SAMPLE_RATE_SYNTH
#define SAMPLE_RATE_SYNTH 48000.0f
#endif
#define MUXSIZE_MIKROTONER 16
#define SHARED_PAR_NUM_OSC 6
#define SHARED_PAR_NUM_LFO 2
#define CURRENT_ADC_UPDATE_THRESHOLD 819
#define SOFT_TAKEOVER_THRESHOLD 64
enum enbSw{
	DISABLE_SW,
	ENABLE_SW
};
enum{
	FALSE = 0,
	TRUE = 1
};
enum modulationMode {
	AMPLITUDE,
	PHASE
};
enum modulationModeLFO {
	FREQUENCY,
	SHAPE
};
enum SelectLFO {
	LFO1,
	LFO2
};
typedef struct {
    double s1L, s2L;
    double s1R, s2R;
}filter_state;

typedef struct {
    enum enbSw enb;
    float f_0;
    float Q;
    float lerpVal_mode;
    float FreqRange;
    float *val_LFO1;
    float *val_LFO2;
    float LFO_depth_freq;
    enum SelectLFO freqLFO_Sel;
}filter_par;

typedef struct {
    float note;
    float vol_osc1;
    float vol_osc2;
    float lerpVal_osc2;
    float shape_osc2;
    float coarse_osc2;
    float fine_osc2;
    double phase_osc2;
    float phaseOffset_osc2;
    float pan_osc2;
    float lerpVal_osc1;
    float shape_osc1;
    float coarse_osc1;
    float fine_osc1;
    double phase_osc1;
    double phaseOffset_osc1;
    float pan_osc1;
    enum enbSw modulEnb;
    enum modulationMode modulMod;
    float modDepth;
    enum enbSw phaseSync;
    float LFO_depth_pitch_osc1;
    enum SelectLFO pitchLFO_Sel_osc1;
    float LFO_depth_pitch_osc2;
    enum SelectLFO pitchLFO_Sel_osc2;
    float LFOlerpDepth_osc1;
    enum SelectLFO lerpLFO_Sel_osc1;
    float LFOlerpDepth_osc2;
    enum SelectLFO lerpLFO_Sel_osc2;
    float *val_LFO1;
    float *val_LFO2;
}osc_par;

typedef struct {
    float lerpVal_lfo1;
    float freq_lfo1;
    float lerpVal_lfo2;
    float freq_lfo2;
    enum enbSw lfo2slave_mode;
    enum modulationModeLFO modulMode;
    float modDepth;
    float val_lfo1;
    float val_lfo2;
    double phase_lfo1;
    double phase_lfo2;
}lfo_par;

typedef struct {
    float x;
    float y;
    float a;
} LP_filter_par;

typedef struct{
    float A;
    float D;
    float S;
    float R;
    float amplitude;
    float keyPressure;
    enum enbSw mode;
    uint8_t keyPressed;
    LP_filter_par value_lp;
} ADSR_par;

typedef struct{
    float vol;
    float color;
    float width;
} noise_par;

float generate_white_noise(uint32_t *state){
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return (int)x / 2147483648.0;
}
void initLPFilterPars(LP_filter_par *p, float freq_cutoff) {
    p->a = (2.0f * M_PI_F * freq_cutoff) / SAMPLE_RATE_SYNTH;

    if (p->a > 1.0f) p->a = 1.0f;
    if (p->a < 0.0f) p->a = 0.0f;
}
void applyLPfilter(LP_filter_par *p) {
    p->y += p->a * (p->x - p->y);
}

float softClip(float x, float threshold, float gain) {
    const float clipStart = threshold * 0.95f;
    const float range = threshold - clipStart;

    if (x > clipStart) {
        float t = (x - clipStart) / range;
        return clipStart + range * tanhf(t * gain);
    } else if (x < -clipStart) {
        float t = (-x - clipStart) / range;
        return -clipStart - range * tanhf(t * gain);
    } else {
        return x;
    }
}

float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

float polyBLEP(float t, float dt) {
    if (t < dt) {
        t /= dt;
        return t + t - t * t - 1.0;
    } else if(t > 1.0 - dt) {
        t = (t - 1.0) / dt;
        return t * t + t + t + 1.0;
    }
    return 0.0;
}
float getLFOValue(float lerpVal, float freq, double *t, float len) {
	if (freq < 0.5f) freq = 1.0f;
	if (freq > 20.0f) freq = 20.0f;
    double phaseInc = freq * len / 2 / SAMPLE_RATE_SYNTH;

    if (lerpVal < 0.0f) lerpVal = 0.0f;
    if (lerpVal > 4.0f) lerpVal = 4.0f;

    float value = 0.0f;
	if(lerpVal <= 1.0f){
		// Sin to Triangle Wave
		float value_sin = sinf(2 * M_PI_F * *t);
		float value_tri = 1.0f- 4.0f* fabsf(*t - 0.5f);
		value = lerp(value_sin, value_tri, lerpVal);
	}
	else if(lerpVal <= 2.0f){
		// Triangle to Ramp Down
		float value_tri = 1.0f- 4.0f* fabsf(*t - 0.5f);
		float value_ramp_down = 2.0f* *t - 1.0;
		value = lerp(value_tri, value_ramp_down, lerpVal - 1.0f);
	}
	else if(lerpVal <= 3.0){
		// Ramp down to Ramp up
		float value_ramp_down = 2.0f* *t - 1.0f;
		float value_ramp_up = -2.0f* *t + 1.0f;
		value = lerp(value_ramp_down, value_ramp_up, lerpVal - 2.0f);
	}
	else if(lerpVal <= 4.0f){
			// Ramp up to Square
			float value_ramp_up = -2.0f * *t + 1.0f;
			float value_sqr = *t < 0.5f ? 1.0f : -1.0f;
			value = lerp(value_ramp_up, value_sqr, lerpVal - 3.0f);
	}
	*t += phaseInc;
	if (*t >= 1.0f) *t -= 1.0f;
	return value;
}

void fullLFOBlock(lfo_par* par, int len) {
	float lerpval_lfo2_temp = par->lerpVal_lfo2;
	float freq_lfo2_temp = par->freq_lfo2;
	par->val_lfo1 = getLFOValue(par->lerpVal_lfo1, par->freq_lfo1, &par->phase_lfo1, len);
	if(par->lfo2slave_mode == ENABLE_SW){
		if(par->modulMode == SHAPE){
			lerpval_lfo2_temp += par->val_lfo1 * par->modDepth * 4.0f;
		}
		else
			freq_lfo2_temp += par->val_lfo1 * par->modDepth * 20.0f;
	}
	par->val_lfo2 = getLFOValue(lerpval_lfo2_temp, freq_lfo2_temp, &par->phase_lfo2, len);
}

float getWaveValue(float t, float lerpVal, float shape, float offset, double phaseInc) {
	t += offset;
	if (t >= 1.0f) {
		t -= 1.0f;
	}
    if (lerpVal < 0.0f) lerpVal = 0.0f;
    if (lerpVal > 3.0f) lerpVal = 3.0f;

    if (shape < 1.0f) shape = 1.0f;
	if (shape > 4.0f) shape = 4.0f;

    float value = 0.0f;
    float ts = 0.0f;
    float clampPoint = 1.0f / shape;
    uint8_t clampWave = 0;
    if(t * shape < 1.0f){
    	ts = t * shape;
    	clampWave = 0;
    }
    else{
    	ts = 1.0f;
    	clampWave = 1;
    }
	if(lerpVal <= 1.0f){
		// Sin to Triangle Wave
		float value_sin = sinf(2 * M_PI_F * ts);
		float value_tri = 1.0f - 4.0f * fabsf(ts - 0.5f);
		if(clampWave){
			value_sin = 0.0f;
			value_tri = -1.0f;
		}
		value = lerp(value_sin, value_tri, lerpVal);
	}
	else if(lerpVal <= 2.0f){
		// Triangle to Sawtooth
		float value_tri = 1.0f - 4.0f * fabsf(ts - 0.5f);
		float value_saw = 2.0f * ts - 1.0f;
		if(clampWave){
			value_tri = -1.0f;
			value_saw = -1.0f;
		}
		value_saw -= polyBLEP(fmod(t + (1.0 - clampPoint), 1.0), phaseInc);
		value = lerp(value_tri, value_saw, lerpVal - 1.0);
	}
	else if(lerpVal <= 3.0){
		float value_saw = 2.0f * ts - 1.0f;
		float value_sqr = ts < 0.5f ? 1.0f : -1.0f;
		if(clampWave){

			value_saw = -1.0f;
			value_sqr = -1.0f;
		}
		value_saw -= polyBLEP(fmod(t + (1.0 - clampPoint), 1.0), phaseInc);
		float midPoint = 0.5f / shape;
		value_sqr += polyBLEP(t, phaseInc);
		value_sqr -= polyBLEP(fmod(t + (1.0 - midPoint), 1.0), phaseInc);
		value = lerp(value_saw, value_sqr, lerpVal - 2.0f);
	}

    return value;
}
void fillWaveBlock(float *buf, int len, osc_par *par) {
	float lerp_LFOVal_osc1 = 0.0f;
	float lerp_LFOVal_osc2 = 0.0f;
	float pitch_LFOVal_osc1 = 0.0f;
	float pitch_LFOVal_osc2 = 0.0f;
	if(par->lerpLFO_Sel_osc1 == LFO2){
		lerp_LFOVal_osc1 = *par->val_LFO2;
	}
	else{
		lerp_LFOVal_osc1 = *par->val_LFO1;
	}
	if(par->lerpLFO_Sel_osc2 == LFO2){
		lerp_LFOVal_osc2 = *par->val_LFO2;
	}
	else{
		lerp_LFOVal_osc2 = *par->val_LFO1;
	}
	if(par->pitchLFO_Sel_osc1 == LFO2){
		pitch_LFOVal_osc1 = *par->val_LFO2;
	}
	else{
		pitch_LFOVal_osc1 = *par->val_LFO1;
	}
	if(par->pitchLFO_Sel_osc2 == LFO2){
		pitch_LFOVal_osc2 = *par->val_LFO2;
	}
	else{
		pitch_LFOVal_osc2 = *par->val_LFO1;
	}
    float freq_osc2 = 110.0f * powf(2.0f, (par->note + par->fine_osc2 + par->coarse_osc2 + pitch_LFOVal_osc2 * par->LFO_depth_pitch_osc2) / 12.0f);
    float freq_osc1 = 110.0f * powf(2.0f, (par->note + par->fine_osc1 + par->coarse_osc1 + pitch_LFOVal_osc1 * par->LFO_depth_pitch_osc1) / 12.0f);
    double phaseInc_osc2 = freq_osc2 / SAMPLE_RATE_SYNTH;
    double phaseInc_osc1 = freq_osc1 / SAMPLE_RATE_SYNTH;
    float lerpVal_osc1_temp = par->lerpVal_osc1;
    float lerpVal_osc2_temp = par->lerpVal_osc2;
    lerpVal_osc1_temp += lerp_LFOVal_osc1 * par->LFOlerpDepth_osc1;
	lerpVal_osc2_temp += lerp_LFOVal_osc2 * par->LFOlerpDepth_osc2;

	float g1L = cosf(par->pan_osc1 * M_PI_2F);
	float g1R = sinf(par->pan_osc1 * M_PI_2F);
	float g2L = cosf(par->pan_osc2 * M_PI_2F);
	float g2R = sinf(par->pan_osc2 * M_PI_2F);
    for (int i = 0; i < len; i = i + 2) {
    	float value = 0;
    	float value_osc1 = 0;
    	float value_osc2 = 0;
    	if(par->modulEnb == ENABLE_SW){
    		if(par->modulMod == PHASE){
				//PM modulation
				if (par->vol_osc1 > 0.05f) {
					if (par->vol_osc2 > 0.05f) {
						value_osc2 = getWaveValue(par->phase_osc2, lerpVal_osc2_temp, par->shape_osc2, par->phaseOffset_osc2, phaseInc_osc2) *par->vol_osc2;
						float modulated_phase = par->phase_osc1 + par->modDepth * value_osc2;
						value = getWaveValue(modulated_phase, lerpVal_osc1_temp, par->shape_osc1, par->phaseOffset_osc1, phaseInc_osc1) *par->vol_osc1;
					}
					else {
						value = getWaveValue(par->phase_osc1, lerpVal_osc1_temp, par->shape_osc1, par->phaseOffset_osc1, phaseInc_osc1) *par->vol_osc1;
					}
				}
				else {
					value = 0.0f;
				}
    		}
    		else if(par->modulMod == AMPLITUDE){
				//AM modulation
				if(par->vol_osc1 > 0.05f){
					value_osc1 = getWaveValue(par->phase_osc1, lerpVal_osc1_temp, par->shape_osc1, par->phaseOffset_osc1, phaseInc_osc1) *par->vol_osc1;
					if(par->vol_osc2 > 0.05f){
					value_osc2 = getWaveValue(par->phase_osc2, lerpVal_osc2_temp, par->shape_osc2, par->phaseOffset_osc2, phaseInc_osc2) *par->vol_osc2;
					value = (1.0f + par->modDepth * value_osc2) * value_osc1;
					}
					else{
						value = value_osc1;
					}
				}
				else{
					value = 0.0f;
				}
    		}
    		buf[i]   = value * g1L;
			buf[i+1] = value * g1R;
    	}
    	else{
    		//Addition, no modulation
    		if(par->vol_osc1 > 0.05f){
    			value_osc1 = getWaveValue(par->phase_osc1, lerpVal_osc1_temp, par->shape_osc1, par->phaseOffset_osc1, phaseInc_osc1) *par->vol_osc1;
    		}
    		else{
    			value_osc1 = 0.0f;
    		}
    		if(par->vol_osc2 > 0.05f){
    		value_osc2 = getWaveValue(par->phase_osc2, lerpVal_osc2_temp, par->shape_osc2, par->phaseOffset_osc2, phaseInc_osc2) *par->vol_osc2;
    		}
    		else{
    			value_osc2 = 0.0f;
    		}
    		value = value_osc1 + value_osc2;
    		buf[i]   = (value_osc1 * g1L) + (value_osc2 * g2L);
			buf[i+1] = (value_osc1 * g1R) + (value_osc2 * g2R);

    	}
        par->phase_osc2 += phaseInc_osc2;
        par->phase_osc1 += phaseInc_osc1;
        if (par->phase_osc1 >= 1.0){
        	par->phase_osc1 -= 1.0;
        	if(par->phaseSync == ENABLE_SW){
				par->phase_osc2 = 0.0f;
			}
        }
        if (par->phase_osc2 >= 1.0) par->phase_osc2 -= 1.0;
    }
}

void fillNoiseBlock(float *in_buf, float *out_buf, int len, noise_par noisePar) {
	static float y_low_mid = 0.0f;
	static float y_low_side = 0.0f;
	static float y_high_mid = 0.0f;
	static float y_high_side = 0.0f;
	static float white_mid_prev = 0.0f;
	static float white_side_prev = 0.0f;
	static uint32_t state_mid  = 0xACE2026;
	static uint32_t state_side = 0x1337BEEF;
	if (noisePar.vol > 1.0f) noisePar.vol = 1.0f;
	if (noisePar.vol < 0.0f) noisePar.vol = 0.0f;
	if (noisePar.color > 0.99f) noisePar.color = 0.99f;
	if (noisePar.color < -0.99f) noisePar.color = -0.99f;
	float color_param = noisePar.color;
	float width_param = noisePar.width;
	float vol_param = noisePar.vol;

	for (int i = 0; i < len; i = i + 2) {
		float white_mid = generate_white_noise(&state_mid);
		float white_side = generate_white_noise(&state_side);
		float colored_mid = white_mid;
		float colored_side = white_side;
		if(color_param >= 0.0f){
			float a = color_param;
			y_low_mid = (1.0f - a) * white_mid + a * y_low_mid;
			colored_mid = y_low_mid;
			y_low_side = (1.0f - a) * white_side + a * y_low_side;
			colored_side = y_low_side;
		}
		else {
			float a = 1.0f + color_param;
			y_high_mid = a * y_high_mid + a * (white_mid - white_mid_prev);
			white_mid_prev = white_mid;
			colored_mid = y_high_mid;
			y_high_side = a * y_high_side + a * (white_side - white_side_prev);
			white_side_prev = white_side;
			colored_side = y_high_side;
		}
		float valueL = (colored_mid + (colored_side * width_param)) * vol_param;
		float valueR = (colored_mid - (colored_side * width_param)) * vol_param;
		out_buf[i] = valueL + in_buf[i];
		out_buf[i+1] = valueR + in_buf[i+1];
	}
}

void StateVariableFilter(float *in_buf, float *out_buf, int len, filter_par *F_par, filter_state *Stt) {
	float freq_LFOVal = 0.0f;
    if (F_par->enb == ENABLE_SW) {
		if (F_par->freqLFO_Sel == LFO2){
			freq_LFOVal = *F_par->val_LFO2;
		}
		else{
			freq_LFOVal = *F_par->val_LFO1;
		}
        float freq = F_par->f_0 + freq_LFOVal * F_par->FreqRange * F_par->LFO_depth_freq;

        if (freq > F_par->FreqRange) freq = F_par->FreqRange;
        if (freq < 10.0f) freq = 10.0f;

        double g = tan((double)M_PI_F * (double)freq / (double)SAMPLE_RATE_SYNTH);
        double k = 1.0 / F_par->Q;

        double a1 = 1.0 / (1.0 + g * (g + k));

        for (uint16_t i = 0; i < len; i += 2) {
            double xL = (double)in_buf[i];
            double xR = (double)in_buf[i+1];

            double hpL = (xL - (g + k) * Stt->s1L - Stt->s2L) * a1;
            double bpL = g * hpL + Stt->s1L;
            double lpL = g * bpL + Stt->s2L;

            double hpR = (xR - (g + k) * Stt->s1R - Stt->s2R) * a1;
			double bpR = g * hpR + Stt->s1R;
			double lpR = g * bpR + Stt->s2R;

            Stt->s1L = g * hpL + bpL;
            Stt->s2L = g * bpL + lpL;

            Stt->s1R = g * hpR + bpR;
			Stt->s2R = g * bpR + lpR;

            double valueL = 0.0;
            double valueR = 0.0;
			if(F_par->lerpVal_mode <= 0.5){
				valueL = lpL;
				valueR = lpR;
			}
			else if(F_par->lerpVal_mode <= 1.5){
				valueL = lerp(lpL, bpL, F_par->lerpVal_mode - 0.5);
				valueR = lerp(lpR, bpR, F_par->lerpVal_mode - 0.5);
			}
			else if(F_par->lerpVal_mode <= 2.0){
				valueL = bpL;
				valueR = bpR;
			}
			else if(F_par->lerpVal_mode <= 3.0){
				valueL = lerp(bpL, hpL, F_par->lerpVal_mode - 2.0);
				valueR = lerp(bpR, hpR, F_par->lerpVal_mode - 2.0);
			}
			else if(F_par->lerpVal_mode <= 3.5){
				valueL = hpL;
				valueR = hpR;
			}
			out_buf[i] = (float)valueL;
			out_buf[i+1] = (float)valueR;
        }
    }
}

void ADSRenveloper(ADSR_par *adsrPar){
	if(adsrPar->A < 0.01) adsrPar->A = 0.01f;
	if(adsrPar->A > 5.0) adsrPar->A = 5.0f;

	if(adsrPar->D < 0.01) adsrPar->D = 0.01f;
	if(adsrPar->D > 5.0) adsrPar->D = 5.0f;

	if(adsrPar->S < 0.0) adsrPar->S = 0.0f;
	if(adsrPar->S > 1.0) adsrPar->S = 1.0f;

	if(adsrPar->R < 0.01) adsrPar->R = 0.01f;
	if(adsrPar->R > 5.0) adsrPar->R = 5.0f;
	static float value = 0.0f;
	static uint8_t step = 0;
	float slide = 0.0f;
	float t_coeff = (BUF_SIZE/4.0f)/(SAMPLE_RATE_SYNTH);
	static uint8_t lastMode = 0;
	if(adsrPar->keyPressed){
		switch(step){
		case 0:
			slide = t_coeff / adsrPar->A;
			if(slide > (1.0f - value)){
				slide = 1.0f - value;
			}
			value += slide;
			if(value >= 1.0f){
				value = 1.0f;
				step = 1;
			}
			break;
		case 1:
			slide = (t_coeff / adsrPar->D) * (1.0f - adsrPar->S);
			if(slide > (value - adsrPar->S)){
				slide = value - adsrPar->S;
			}
			if(value > adsrPar->S){
				value -= slide;
			}
			else{
				value = adsrPar->S;
			}
			break;
		}
	}
	else{
		slide = t_coeff / adsrPar->R;
		if(slide > value){
			slide = value;
		}
		value -= slide;
		if(value <= 0.0f){
		    value = 0.0f;
		}
		step = 0;
	}
	if(adsrPar->mode == ENABLE_SW){
		if(!lastMode){
			value = 0.0f;
			lastMode = 1;
		}
		adsrPar->value_lp.x = value;
		applyLPfilter(&adsrPar->value_lp);
		adsrPar->amplitude = adsrPar->value_lp.y;
	}
	else{
		if(lastMode){
			value = 0.0f;
			lastMode = 0;
		}
		adsrPar->amplitude = adsrPar->keyPressure;
	}
	if(adsrPar->amplitude < 0.0f) adsrPar->amplitude = 0.0f;
	if(adsrPar->amplitude > 1.0f) adsrPar->amplitude = 1.0f;
}
