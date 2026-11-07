/*
 * sounds.h
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#ifndef SOUNDS_H_
#define SOUNDS_H_

#include "main.h"

void playStartupTune(void);
void playBrushedStartupTune(void);

void setVolume(uint8_t volume);

void requestDshotBeacon(uint8_t command);
void requestRuntimeTone(uint8_t tone, uint8_t repeats);
// Short bridge ownership transactions; restore the caller's interrupt mask.
uint32_t lockMotorOutputs(void);
void unlockMotorOutputs(uint32_t mask);
void cancelRuntimeTone(void);
void serviceRuntimeTone(void);

#endif /* SOUNDS_H_ */
