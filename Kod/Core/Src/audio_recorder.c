/*
 * audio_recorder.c
 *
 *  Created on: Nov 5, 2025
 *      Author: 278315
 */

#include "audio_recorder.h"
#include <string.h>
#include <stdio.h>
int32_t recBuff[AUDIO_BUF];
volatile uint32_t samplesSent = 0;
int16_t audioChunk[AUDIO_BUF/2];
uint8_t wavHeader[44];


void createWavHeader(uint8_t *header, uint32_t sampleRate, uint32_t dataSize)
{
	memset(header, 0, 44);
	uint32_t chunkSize= 36 + dataSize;
	uint32_t subchunk1Size= 16;
	uint16_t audioFormat= 1;
	uint16_t numChannels= 1;
	uint16_t bitsPerSample= 16;
	uint32_t byteRate= sampleRate * numChannels * bitsPerSample / 8;
	uint16_t blockAlign= numChannels * bitsPerSample / 8;

	memcpy(header + 0,"RIFF", 4);
	memcpy(header + 4,&chunkSize, 4);
	memcpy(header + 8,"WAVE", 4);

	memcpy(header + 12,"fmt ", 4);
	memcpy(header + 16,&subchunk1Size, 4);
	memcpy(header + 20,&audioFormat, 2);
	memcpy(header + 22,&numChannels, 2);
	memcpy(header + 24,&sampleRate, 4);
	memcpy(header + 28,&byteRate, 4);
	memcpy(header + 32,&blockAlign, 2);
	memcpy(header + 34,&bitsPerSample, 2);

	memcpy(header + 36,"data", 4);
	memcpy(header + 40,&dataSize, 4);
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
    for (int i = 0; i < AUDIO_BUF/2; i++)
    	audioChunk[i] = (int16_t)(recBuff[i]);

    HAL_UART_Transmit(&huart2, (uint8_t*)audioChunk, AUDIO_BUF, HAL_MAX_DELAY);
    samplesSent += (AUDIO_BUF/2);

    if (samplesSent >= SAMPLE_RATE * TOTAL_SECONDS)
    	HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
    for (int i = AUDIO_BUF/2; i < AUDIO_BUF; i++)
    	audioChunk[i - AUDIO_BUF/2] = (int16_t)(recBuff[i]);

    HAL_UART_Transmit(&huart2, (uint8_t*)audioChunk, AUDIO_BUF, HAL_MAX_DELAY);
    samplesSent += (AUDIO_BUF/2);

    if (samplesSent >= SAMPLE_RATE * TOTAL_SECONDS)
    	HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
}
