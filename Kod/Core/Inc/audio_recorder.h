/*
 * audio_header.h
 *
 *  Created on: Nov 5, 2025
 *      Author: 278315
 */

#ifndef INC_AUDIO_RECORDER_H_
#define INC_AUDIO_RECORDER_H_

#include "main.h"
#include <stdint.h>

#define AUDIO_BUF 			1024
#define SAMPLE_RATE      	22050
#define TOTAL_SECONDS		30
#define RMS_THRESHOLD 		16000.0f

extern UART_HandleTypeDef huart2;
extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;

extern int32_t recBuff[AUDIO_BUF];
extern volatile uint32_t samplesSent;
extern uint8_t wavHeader[44];
extern int16_t audioChunk[AUDIO_BUF/2];
extern volatile uint8_t isRecordingActive;



void createWavHeader(uint8_t *header, uint32_t sampleRate);
void AudioRecorderInit();

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *h);
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *h);
void DFSDMCallbackHandler(uint8_t isBuffFull);

#endif /* INC_AUDIO_RECORDER_H_ */
