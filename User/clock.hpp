
/*
 * clock.cpp
 *
 *  Created on: Jul 20, 2018
 *      Author: Douwe
 *
 *       Clock class syncs the master clock (TIM2) to an incoming rising edge signal
 *      or generates it's own interrupt ussing CCR1.
 *
 *      TIM2's period is then subdivided tp the slave clock (TIM5) which in its IRQ
 *      updates the sequencer step
 *
 *      see stm32f4xx_it.cpp for the routines in use
 *
 */
#include "main.h"
#include "stm32f4xx.h"


#ifndef CLOCK_HPP_
#define CLOCK_HPP_


enum ClockSource{
		INTERNAL,
		EXTERNAL
};

class Clock{
public:
	TIM_HandleTypeDef* masterTimer;
	TIM_HandleTypeDef* slaveTimer;
	ClockSource clockSource;
	bool lockState;

	volatile unsigned int period, subDiv, subMult, master, sub;
	unsigned int averagedPeriod;
	unsigned int lastCounterValue;

	int averagingWindow;
	int sampleIndex;


	// Constructor
	Clock(void);
	// Destructor
	~Clock(void);

	void setTimer(TIM_HandleTypeDef* master, TIM_HandleTypeDef* slave);

	// Add a sampled period to average, returns 0 if result is out of bounds which resets the buffer.
	// Returns averaged period if the buffer has at least 2 samples.
	unsigned int addPeriodSample(unsigned int periodSample);

	// Set a period manually
	void setPeriod(void);
	void setPeriod(unsigned int value);

	volatile unsigned int getPeriod(void);

	void setSlaveDivision(unsigned int division);
	void setSlaveMultiplier(unsigned int multiplier);
	void setSource(ClockSource source);

	// Increment master clock;
	void masterTick(void);
	void subTick(void);

	// Is the clock locked?
	void reset(void);
	void lock(void);
	void unlock(void);
	bool isLocked(void);

private:
	float acceptableOutlierPercentage;
	int acceptableDeviation;

	bool sampleBufferFull;
	unsigned int sampleBuffer[32];


};




#endif /* CLOCK_HPP_ */
