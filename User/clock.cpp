/*
 * clock.cpp
 *
 *  Created on: Jul 20, 2018
 *      Author: Douwe
 *
 *       Clock class syncs the master clock (TIM2) to an incoming rising edge signal
 *      or generates it's own interrupt using CCR1.
 *
 *
 *      TIM2's period is then subdivided to the slave clock (TIM5) which in its IRQ
 *      updates the sequencer's step state and outputs
 *
 *      see stm32f4xx_it.cpp for the routines in use
 *
 *      Improvements to be made:
 *      - Add swing
 *      - Add midi sync
 *      - Add i2c sync
 *      - Average input clock for stability?
 *
 */
#include "clock.hpp"
#include "stm32f4xx.h"

// Constructor
Clock::Clock(){
	averagingWindow = 23;
	acceptableOutlierPercentage = 0.1;
	subMult = 1;
	;
}
// Destructor
Clock::~Clock(void){
	;
}
void Clock::reset(void){
	sampleIndex = 0; sampleBufferFull = false;
	for(int i = 0; i < averagingWindow; i++){
		sampleBuffer[i] = 0;
	}
}
unsigned int Clock::addPeriodSample(unsigned int periodSample){

	// If the sample buffer has not overflown and this is one of the first samples always add it
	if((sampleIndex  == 0) && (sampleBufferFull == false)){
		sampleBuffer[sampleIndex] = periodSample;
		acceptableDeviation = periodSample*acceptableOutlierPercentage;
		averagedPeriod = periodSample;
		sampleIndex++;
	}
	else if(sampleIndex < averagingWindow){
		// Check if the sample falls within the acceptable deviation bounds
		if(((periodSample+acceptableDeviation)>averagedPeriod)
			&& ((periodSample-acceptableDeviation)<averagedPeriod)){

			// Add the sampled clock period to the cyclic buffer
			sampleBuffer[sampleIndex] = periodSample;
			sampleIndex++;
		}
		else{
			// Reset the buffer, store the sample anyway.
			sampleIndex = 0; sampleBufferFull = false;
			for(int i = 0; i < averagingWindow; i++){
				sampleBuffer[i] = 0;
			}
			sampleBuffer[sampleIndex] = periodSample;
			//sampleIndex++;
		}
	}
	else{sampleIndex = 0; sampleBufferFull = true;
			sampleBuffer[sampleIndex] = periodSample;
			sampleIndex++;
	}

	// Check if there is more than 1 sample, then average and calculate bounds. If there is only 1 sample return false.
	// We need 2 edges to capture a period length...
	if(sampleBufferFull){
		unsigned int temp = 0;
		for(int i = 0; i < averagingWindow ; i++){
			temp+=sampleBuffer[i];
		}
		averagedPeriod = temp/(averagingWindow);

		// Calculate the bounds for the next sample<
		acceptableDeviation = averagedPeriod*acceptableOutlierPercentage;
		return averagedPeriod;
	}
	else if(sampleIndex > 1){
			unsigned int temp = 0;
			for(int i = 0; i < sampleIndex; i++){
				temp+=sampleBuffer[i];
			}
			averagedPeriod = temp/(sampleIndex);

			acceptableDeviation = averagedPeriod*acceptableOutlierPercentage;
			return 0;
		}
	else{
		// If only 1 sample has been captured:

		return 0;
	}
}
void Clock::masterTick(void){
		master++;

}
void Clock::subTick(void){
	if(++sub < subDiv){
		;
	}
	else{
	   sub = 0;
	}
}
// This should happen in the constructor!
void Clock::setTimer(TIM_HandleTypeDef* master, TIM_HandleTypeDef* slave){
	masterTimer = master;
	slaveTimer = slave;
}

// Implement averaging routine! and Reset on master pulse.
// Maybe inline these functions!
void Clock::lock(){
	if(!lockState){
		lockState = true;
		sub = 0;
	}
}
void Clock::unlock(){
	lockState = false;
}
bool Clock::isLocked(void){
	return lockState;
}
void Clock::setPeriod(void){
	if(clockSource == INTERNAL){
		masterTimer->Instance->CCR2 = period;
	}
	else if(clockSource == EXTERNAL){
	     period = masterTimer->Instance->CCR1;
	     masterTimer->Instance->CCR2 = period;
	}
}
void Clock::setPeriod(unsigned int value){
	if(clockSource == INTERNAL){
		period = value;
		masterTimer->Instance->CCR2 = period;
	}
	else if(clockSource == EXTERNAL){
		period = value;
		masterTimer->Instance->CCR2 = period;
	}
}

void Clock::setSource(enum ClockSource source){
	clockSource = source;
	if(source == INTERNAL){
		// Do not use the external trigger
		HAL_TIM_IC_Stop_IT(masterTimer, TIM_CHANNEL_1);
		HAL_TIM_OC_Start_IT(masterTimer, TIM_CHANNEL_2);

	}
	else if(source == EXTERNAL){
		HAL_TIM_IC_Start_IT(masterTimer, TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(masterTimer, TIM_CHANNEL_1);
	}
}
volatile unsigned int Clock::getPeriod(void){
	return period;
}
void Clock::setSlaveMultiplier(unsigned int multiplier){
	subMult = multiplier;


}

void Clock::setSlaveDivision(unsigned int division){
	subDiv = division;
	// Bit hacky, to make sure there are only 'division' amounts of pulses in a period.
	// This may be necessary if a shoddy clock input is used
	slaveTimer->Instance->CNT = 0;
	slaveTimer->Instance->ARR = (period+50)/subDiv;
	slaveTimer->Instance->CCR2 = 1000;

	// slaveTimer->Instance->ARR = ((period)/division);//+(period%division);
}
