/*
 * audio_recorder.c
 *
 *  Created on: Nov 5, 2025
 *      Author: 278315
 */

#include "audio_recorder.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
int32_t recBuff[AUDIO_BUF];
volatile uint32_t samplesSent = 0;
int16_t audioChunk[AUDIO_BUF/2];
uint8_t wavHeader[44];
volatile uint8_t isRecordingActive = 0;

void createWavHeader(uint8_t *header, uint32_t sampleRate)
{
	memset(header, 0, 44);
	uint32_t fmtChunkSize= 16;
	uint16_t audioFormat= 1;
	uint16_t numChannels= 1;
	uint16_t bitsPerSample= 16;
	uint32_t byteRate= sampleRate * numChannels * bitsPerSample / 8;
	uint16_t sampleAlign= numChannels * bitsPerSample / 8;
	uint32_t dataSize= sampleRate*TOTAL_SECONDS*numChannels*bitsPerSample/8;
	uint32_t wavSize= 36 + dataSize;

	memcpy(header + 0,"RIFF", 4);
	memcpy(header + 4,&wavSize, 4);
	memcpy(header + 8,"WAVE", 4);

	memcpy(header + 12,"fmt ", 4);
	memcpy(header + 16,&fmtChunkSize, 4);
	memcpy(header + 20,&audioFormat, 2);
	memcpy(header + 22,&numChannels, 2);
	memcpy(header + 24,&sampleRate, 4);
	memcpy(header + 28,&byteRate, 4);
	memcpy(header + 32,&sampleAlign, 2);
	memcpy(header + 34,&bitsPerSample, 2);

	memcpy(header + 36,"data", 4);
	memcpy(header + 40,&dataSize, 4);
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
	uint64_t sum=0;
	int16_t sample=0;
	float rms=0;
    for (int i = 0; i < AUDIO_BUF/2; i++){
    	sample = (int16_t)(recBuff[i]);
    	audioChunk[i] = sample;
    	sum += (uint32_t) sample*sample;
    }
    rms = sqrtf(sum/(AUDIO_BUF/2));
    if (!isRecordingActive && rms >= RMS_THRESHOLD){
    	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    	isRecordingActive = 1;
        createWavHeader(wavHeader,SAMPLE_RATE);
        HAL_UART_Transmit(&huart2, wavHeader, 44, HAL_MAX_DELAY);
    }

    if (isRecordingActive) {
		HAL_UART_Transmit(&huart2, (uint8_t*)audioChunk, AUDIO_BUF, HAL_MAX_DELAY);
		samplesSent += (AUDIO_BUF/2);

		if (samplesSent >= SAMPLE_RATE * TOTAL_SECONDS){
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
			HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
		}
    }
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
	uint64_t sum=0;
	int16_t sample=0;
	float rms=0;
    for (int i = AUDIO_BUF/2; i < AUDIO_BUF; i++){
    	sample = (int16_t)(recBuff[i]);
    	audioChunk[i - AUDIO_BUF/2] = sample;
    	sum += (uint32_t) sample*sample;
	}
    rms = sqrtf(sum/(AUDIO_BUF/2));
    if (!isRecordingActive && rms >= RMS_THRESHOLD){
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        isRecordingActive = 1;
        createWavHeader(wavHeader,SAMPLE_RATE);
        HAL_UART_Transmit(&huart2, wavHeader, 44, HAL_MAX_DELAY);
    }

    if (isRecordingActive){
		HAL_UART_Transmit(&huart2, (uint8_t*)audioChunk, AUDIO_BUF, HAL_MAX_DELAY);
		samplesSent += (AUDIO_BUF/2);

		if (samplesSent >= SAMPLE_RATE * TOTAL_SECONDS){
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
			HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
		}
    }
}
