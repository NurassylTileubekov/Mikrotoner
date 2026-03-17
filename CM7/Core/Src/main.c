/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "dsp_adsr.h"
#include "dsp_config.h"
#include "dsp_enum.h"
#include "dsp_filter.h"
#include "dsp_lfo.h"
#include "dsp_noise.h"
#include "dsp_osc.h"
#include "dsp_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
float osc_buf[BUF_SIZE];
float noise_buf[BUF_SIZE];
float f1_buf[BUF_SIZE];

int32_t i2s_buf[BUF_SIZE];

filter_par_t filter_par;
filter_state_t filter_state;

osc_par_t osc_par;
osc_state_t osc_state;

lfo_par_t lfo_par;
lfo_state_t lfo_state;

adsr_par_t adsr_par;
adsr_state_t adsr_state;

noise_par_t noise_par;
noise_state_t noise_state;

float master_vol = 0.0f;
float threshold = 0.95f;
float gain = 0.8f;

__attribute__ ((section (".ADC3buf"), used)) volatile uint16_t adc3_vals[4];
volatile uint16_t adc2_vals[2] = {0};
uint16_t adc_vals[37] = {0};
uint16_t mux_adc_vals[15];
uint8_t mux_sel[4] = {0};
uint8_t mux_inc = 0;
uint8_t switch_vals[16] = {0};

uint8_t sel_osc = 0;
uint8_t sel_lfo = 0;
uint8_t sel_page = 0;
uint8_t sel_pan = 0;
uint8_t last_sel_osc = 0;
uint8_t last_sel_page = 0;
uint8_t last_sel_lfo = 0;
uint8_t last_sel_pan = 0;
uint8_t set_curr_adc_vals_osc[SHARED_PAR_NUM_OSC] = {1};
uint8_t set_curr_adc_vals_lfo[SHARED_PAR_NUM_LFO] = {1};
uint8_t set_curr_adc_vals_page[6] = {1};
uint8_t set_curr_adc_vals_pan[2] = {1};

lp_filter_par_t adc_f_par[37];

uint8_t semitone_mode = 0;
float portamento_time_s = 0.5f;
volatile uint8_t adc2_flag = 0;
volatile uint8_t adc3_flag = 0;

volatile uint8_t i2s_process_half_buf = 0;
volatile uint16_t i2s_underrun = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_BDMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void update_parameters(){
	float adc_f_vals[37];
	for(int i = 0; i < 37; i++){
		adc_f_par[i].x = adc_vals[i];
		apply_lp_filter(&adc_f_par[i]);
		adc_f_vals[i] = adc_f_par[i].y;
	}
	osc_par.lerp_val_osc1 = adc_f_vals[0]/4095.0f *3.0f;
	osc_par.shape_osc1 = adc_f_vals[1]/4095.0f *3.0f + 1.0f;
	float normalized = adc_f_vals[2]/4095.0f *24.0f;
	float rounded = roundf(normalized);
	if(fabsf(normalized - rounded) < 0.2f){
		osc_par.coarse_osc1 = rounded;
	}
	osc_par.fine_osc1 = adc_f_vals[3]/4095.0f *2.0f - 1.0f;
	osc_par.lfo_depth_pitch_osc1 = adc_f_vals[4]/4095.0f *12.0f;
	osc_par.lfo_lerp_depth_osc1 = adc_f_vals[5]/4095.0f *3.0f;

	osc_par.lerp_val_osc2 = adc_f_vals[6]/4095.0f *3.0f;
	osc_par.shape_osc2 = adc_f_vals[7]/4095.0f *3.0f + 1.0f;
	normalized =adc_f_vals[8]/4095.0f *24.0f;
	rounded = roundf(normalized);
	if(fabsf(normalized - rounded) < 0.2f){
		osc_par.coarse_osc2 = rounded;
	}
	osc_par.fine_osc2 = adc_f_vals[9]/4095.0f *2.0f - 1.0f;
	osc_par.lfo_depth_pitch_osc2 = adc_f_vals[10]/4095.0f *12.0f;
	osc_par.lfo_lerp_depth_osc2 = adc_f_vals[11]/4095.0f *3.0f;

	lfo_par.lerp_val_lfo1 = adc_f_vals[12]/4095.0f *4.0f;
	lfo_par.freq_lfo1 = adc_f_vals[13]/4095.0f *19.0f + 1.0f;

	lfo_par.lerp_val_lfo2 = adc_f_vals[14]/4095.0f *4.0f;
	lfo_par.freq_lfo2 = adc_f_vals[15]/4095.0f *19.0f + 1.0f;

	osc_par.mod_depth = adc_f_vals[16]/4095.0f;

	portamento_time_s = adc_f_vals[17]/4095.0f * 0.9f + 0.1f;
	noise_par.vol = (adc_f_vals[22] - 20)/4075.0f;
	normalized = adc_f_vals[21] / 4095.0f;
	filter_par.f_0 = 20.0f * powf(1000.0f, normalized);
	filter_par.q = adc_f_vals[20]/4095.0f *49.0f + 1.0f;
	filter_par.lerp_val_mode = adc_f_vals[19]/4095.0f *3.5F;
	filter_par.lfo_depth_freq = adc_f_vals[18]/4095.0f;

	noise_par.color = adc_f_vals[23]/4095.0f *2.0f - 1.0f;
	noise_par.width = adc_f_vals[28]/4095.0f;
	adsr_par.a = adc_f_vals[27]/4095.0f * 5.0f;
	adsr_par.d = adc_f_vals[26]/4095.0f * 5.0f;
	adsr_par.s = adc_f_vals[25]/4095.0f;
	adsr_par.r = adc_f_vals[24]/4095.0f * 5.0f;

	lfo_par.mod_depth = adc_f_vals[29]/4095.0f;

	master_vol = (adc_f_vals[34] - 20)/4075.0f;
	osc_par.pan_osc1 = (adc_f_vals[33] - 20)/4075.0f;
	osc_par.pan_osc2 = (adc_f_vals[32] - 20)/4075.0f;

	osc_par.vol_osc1 = (adc_f_vals[31] - 20)/4075.0f;
	osc_par.vol_osc2 = (adc_f_vals[30] - 20)/4075.0f;

	//adsr_par.key_pressure = (adc_f_vals[36] - 300.0f)/3590.0f;
	if(adsr_par.key_pressure >= 0.0f){
		adsr_par.key_pressed = 1;
	}
	else{
		adsr_par.key_pressed = 0;
	}
	static uint8_t last_key_pressed = 0;
	static float init_diff = 0.0f;
	static float target_note = 0.0f;

	if(adsr_par.key_pressed){
	    if(semitone_mode){
	        normalized = adc_f_vals[35] / 4095.0f * 24.0f;
	        rounded = roundf(normalized);
	        if(last_key_pressed){
	            if(rounded != target_note) {
	                init_diff = rounded - osc_par.note;
	                target_note = rounded;
	            }
	            float diff = target_note - osc_par.note;
	            if(fabsf(diff) > 0.001f){
	                if(portamento_time_s < 0.01f) portamento_time_s = 0.01f;
	                float portamento_step = (init_diff * (BUF_SIZE / 4.0f)) / (SAMPLE_RATE_SYNTH * portamento_time_s);
	                if(fabsf(diff) < fabsf(portamento_step)){
	                    osc_par.note = target_note;
	                }
	                else{
	                	osc_par.note += portamento_step;
	                }
	            }
	        }
	        else{
	            //osc_par.note = rounded; commented out for testing without the membrane pot
	            target_note = rounded;
	            init_diff = 0.0f;
	        }
	    }
	    else{
	        //osc_par.note = adc_f_vals[35] / 4095.0f * 24.0f; commented out for testing without the membrane pot
	    }
	    last_key_pressed = 1;
	}
	else{
	    last_key_pressed = 0;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == ADC3){
		adc3_flag = TRUE;
	}
	if(hadc->Instance == ADC2){
		adc2_flag = TRUE;
	}

}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	if(i2s_process_half_buf != 0){
		i2s_underrun++;
	}
	i2s_process_half_buf = 1;
}
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	if(i2s_process_half_buf != 0){
		i2s_underrun++;
	}
	i2s_process_half_buf = 2;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_I2S2_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) {
      Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) {
      Error_Handler();
  }
  fill_wave_block((float*)i2s_buf, BUF_SIZE, &osc_par, &osc_state);
  HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)i2s_buf, BUF_SIZE);
  i2s_process_half_buf = 0;
  osc_par.val_lfo1 = &lfo_par.val_lfo1;
  osc_par.val_lfo2 = &lfo_par.val_lfo2;
  filter_par.val_lfo1 = &lfo_par.val_lfo1;
  filter_par.val_lfo2 = &lfo_par.val_lfo2;

  for(int i = 0; i < 36; i++){
	  init_lp_filter_pars(&adc_f_par[i], 500.0f);
  }
  init_lp_filter_pars(&adc_f_par[30], 100.0f);
  init_lp_filter_pars(&adc_f_par[31], 100.0f);
  init_lp_filter_pars(&adc_f_par[34], 100.0f);
  init_lp_filter_pars(&adc_f_par[36], 100.0f);
  init_lp_filter_pars(&adsr_state.value_lp, 200.0f);

  init_adsr_state(&adsr_state);
  init_filter_state(&filter_state);
  set_filter_range(&filter_par, 20000.0f);
  init_lfo_state(&lfo_state);
  init_noise_state(&noise_state);
  init_osc_state(&osc_state);

  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&adc3_vals, 4);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc2_vals, 2);
  mux_inc = 0;
  mux_sel[0] = 0;
  mux_sel[1] = 0;
  mux_sel[2] = 0;
  mux_sel[3] = 0;
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, mux_sel[0]);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, mux_sel[1]);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, mux_sel[2]);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, mux_sel[3]);
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	switch(i2s_process_half_buf) {
		case 1:
			update_parameters();
			full_lfo_block(&lfo_par, &lfo_state, BUF_SIZE/2);
			adsr_enveloper(&adsr_par, &adsr_state);
			fill_wave_block(osc_buf, BUF_SIZE/2, &osc_par, &osc_state);
			fill_noise_block(osc_buf, noise_buf, BUF_SIZE/2, noise_par, &noise_state);
			state_variable_filter(noise_buf, f1_buf, BUF_SIZE/2, &filter_par, &filter_state);
			for (int i = 0; i < BUF_SIZE/2; i++) {
				float value = 0.0f;
				if(filter_par.enb){
					value = f1_buf[i];
				}
				else{
					value = noise_buf[i];
				}
				//value = value * master_vol * adsr_state.amplitude; commented out for testing without the pressure sensor
				value = value * master_vol;
				value = soft_clip(value, threshold, gain);
				value *= 2147483648.0f;
				if (value > 2147483647.0f) value = 2147483647.0f;
				if (value < -2147483648.0f) value = -2147483648.0f;
				i2s_buf[i] = (int32_t)value;
			}
			i2s_process_half_buf = 0;
			break;
		case 2:
			update_parameters();
			full_lfo_block(&lfo_par, &lfo_state, BUF_SIZE/2);
			adsr_enveloper(&adsr_par, &adsr_state);
			fill_wave_block(osc_buf + BUF_SIZE/2, BUF_SIZE/2, &osc_par, &osc_state);
			fill_noise_block(osc_buf + BUF_SIZE/2, noise_buf + BUF_SIZE/2, BUF_SIZE/2, noise_par, &noise_state);
			state_variable_filter(noise_buf + BUF_SIZE/2, f1_buf + BUF_SIZE/2, BUF_SIZE/2, &filter_par, &filter_state);
			for (int i = BUF_SIZE/2; i < BUF_SIZE; i++) {
				float value = 0.0f;
				if(filter_par.enb){
					value = f1_buf[i];
				}
				else{
					value = noise_buf[i];
				}
				//value = value * master_vol * adsr_state.amplitude; commented out for testing without the pressure sensor
				value = value * master_vol;
				value = soft_clip(value, threshold, gain);
				value *= 2147483648.0f;
				if (value > 2147483647.0f) value = 2147483647.0f;
				if (value < -2147483648.0f) value = -2147483648.0f;
				i2s_buf[i] = (int32_t)value;
			}
			i2s_process_half_buf = 0;
			break;
		default:
			if(adc3_flag == TRUE){
				adc_vals[34] = adc3_vals[1];
				static int first_iter1 = 1;
				static int first_iter2 = 1;
				mux_adc_vals[mux_inc] = adc3_vals[0];
				switch_vals[mux_inc] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8);
				if(mux_inc < 15){
					mux_inc++;
				}
				else{
					sel_osc                    = switch_vals[0];
					osc_par.phase_sync         = switch_vals[1];
					osc_par.modul_enb          = switch_vals[2];
					osc_par.modul_mod          = switch_vals[3];
					osc_par.pitch_lfo_sel_osc1 = switch_vals[4];
					osc_par.lerp_lfo_sel_osc1  = switch_vals[5];
					osc_par.pitch_lfo_sel_osc2 = switch_vals[6];
					osc_par.lerp_lfo_sel_osc2  = switch_vals[7];
					filter_par.enb             = switch_vals[8];
					filter_par.freq_lfo_sel    = switch_vals[9];
					sel_lfo                    = switch_vals[10];
					lfo_par.lfo2_slave_mode    = switch_vals[11];
					lfo_par.modul_mode         = switch_vals[12];
					sel_page	               = switch_vals[13];
					semitone_mode              = switch_vals[14];
					adsr_state.mode            = switch_vals[15];
					if(first_iter2){
						last_sel_osc = sel_osc;
						last_sel_lfo = sel_lfo;
						last_sel_page = sel_page;
						for(int i = 0; i < 6; i++){
							set_curr_adc_vals_osc[i] = 1;
						}
						for(int i = 0; i < 2; i++){
							set_curr_adc_vals_lfo[i] = 1;
						}
						for(int i = 0; i < 5; i++){
							set_curr_adc_vals_page[i] = 1;
						}
						first_iter2 = 0;
					}
					if(last_sel_osc != sel_osc){
						for(int i = 0; i < 6; i++){
							set_curr_adc_vals_osc[i] = 0;
						}
					}
					for(int i = 0; i < 6; i++){
						if(abs((int)adc_vals[i+sel_osc*6] - (int)mux_adc_vals[i]) <= SOFT_TAKEOVER_THRESHOLD || set_curr_adc_vals_osc[i]){
							adc_vals[i+sel_osc*6] = mux_adc_vals[i];
							set_curr_adc_vals_osc[i] = 1;
						}
					}
					last_sel_osc = sel_osc;

					if(last_sel_lfo != sel_lfo){
						for(int i = 0; i < 2; i++){
							set_curr_adc_vals_lfo[i] = 0;
						}
					}
					for(int i = 0; i < 2; i++){
						if(abs((int)adc_vals[i+sel_lfo*2+12] - (int)mux_adc_vals[i+6]) <= SOFT_TAKEOVER_THRESHOLD || set_curr_adc_vals_lfo[i]){
							adc_vals[i+sel_lfo*2+12] = mux_adc_vals[i+6];
							set_curr_adc_vals_lfo[i] = 1;
						}
					}
					last_sel_lfo = sel_lfo;

					adc_vals[16] = mux_adc_vals[8];

					if(last_sel_page != sel_page){
						for(int i = 0; i < 6; i++){
							set_curr_adc_vals_page[i] = 0;
						}
					}
					for(int i = 0; i < 6; i++){
						if(abs((int)adc_vals[i+sel_page*6+17] - (int)mux_adc_vals[i+9]) <= SOFT_TAKEOVER_THRESHOLD || set_curr_adc_vals_page[i]){
							adc_vals[i+sel_page*6+17] = mux_adc_vals[i+9];
							set_curr_adc_vals_page[i] = 1;
						}
					}
					last_sel_page = sel_page;
					mux_inc = 0;

					adc_vals[29] = mux_adc_vals[15];
				}
				if(first_iter1){
					last_sel_pan = sel_page;
					set_curr_adc_vals_pan[0] = 1;
					set_curr_adc_vals_pan[1] = 1;
					first_iter1 = 0;
				}
				if(last_sel_pan != sel_page){
					set_curr_adc_vals_pan[0] = 0;
					set_curr_adc_vals_pan[1] = 0;
				}
				for(int i = 0; i < 2; i++){
					if(abs((int)adc_vals[i+sel_page*2+30] - (int)adc3_vals[i+2]) <= SOFT_TAKEOVER_THRESHOLD || set_curr_adc_vals_pan[i]){
						adc_vals[i+sel_page*2+30] = adc3_vals[i+2];
						set_curr_adc_vals_pan[i] = 1;
					}
				}
				last_sel_pan = sel_page;
				mux_sel[0] = mux_inc & 1;
				mux_sel[1] = (mux_inc>>1) & 1;
				mux_sel[2] = (mux_inc>>2) & 1;
				mux_sel[3] = (mux_inc>>3) & 1;
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, mux_sel[0]);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, mux_sel[1]);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, mux_sel[2]);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, mux_sel[3]);
				adc3_flag = FALSE;
			}
			if(adc2_flag == TRUE){
				adc_vals[36] = adc2_vals[1];
				if(adc_vals[36] > 300)
					adc_vals[35] = adc2_vals[0];
				adc2_flag = FALSE;
			}
			break;
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 50;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = 256;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = ENABLE;
  hadc3.Init.Oversampling.Ratio = 128;
  hadc3.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_7;
  hadc3.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc3.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_BDMA_CLK_ENABLE();

  /* DMA interrupt init */
  /* BDMA_Channel0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, S0_Pin|S3_Pin|S2_Pin|S1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : S0_Pin S3_Pin S2_Pin S1_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S3_Pin|S2_Pin|S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
