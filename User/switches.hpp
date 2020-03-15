/*
 * ui.hpp
 *
 *  Created on: Feb 4, 2019
 *      Author: delta
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include <vector>
#include <cstring>
#include <stdlib.h>


#ifndef SWITCHES_HPP_
#define SWITCHES_HPP_

#define NUM_OF_REGISTERS 6
#define RCLK_GPIO_Port SPI1_RCLK_GPIO_Port
#define RCLK_Pin SPI1_RCLK_Pin

enum PWM_MODE{
	PWM_OFF = 0,
	PWM_DIM = 5,
	PWM_HALF = 32,
	PWM_FULL = 64
};

enum PULSE_MODE{
	PULSE_MODE_OFF = 0,
	PULSE_MODE_16TH = 2,
	PULSE_MODE_8TH = 4,
	PULSE_MODE_4TH = 8,
	PULSE_MODE_2ND = 16
};

enum SWITCH_MODE{
	SWITCH_MODE_STEP,
	SWITCH_MODE_CONTROL
};

enum CONTROL_SWITCH{
	CONTROL_SWITCH_SCENE = 0,
	CONTROL_SWITCH_PART = 1,
	CONTROL_SWITCH_LANE = 2,
	CONTROL_SWITCH_STEP = 3,
	CONTROL_SWITCH_COPY = 4,
	CONTROL_SWITCH_PASTE = 5,
	CONTROL_SWITCH_SELECT = 6,
	CONTROL_SWITCH_MOVE = 7,
	CONTROL_SWITCH_START_STOP = 8,
	CONTROL_SWITCH_HOLD = 9,
	CONTROL_SWITCH_SHIFT = 10,
	CONTROL_SWITCH_MODE = 11,
	CONTROL_SWITCH_A = 12,
	CONTROL_SWITCH_B = 13,
	CONTROL_SWITCH_C = 14,
	CONTROL_SWITCH_D = 15,
};

class Encoder{
public:
	bool debouncedStateA, debouncedStateB, debouncedStateSw;
	bool encoderChanged, switchChanged;

	int16_t counter;
	Encoder(uint8_t nStates,GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin_A, uint16_t GPIO_Pin_B, uint16_t GPIO_Pin_Sw);
	~Encoder();
	bool update(void);
	bool hasChanged(void);
	bool wasPressed(void);
	bool wasReleased(void);
	int16_t readAndResetCounter(void);

private:
	GPIO_TypeDef* port;
	uint16_t pinA, pinB, pinSw;
	uint8_t numOfStates;
	uint8_t index;
	uint8_t bufferA, bufferB, bufferSw;
	uint8_t fullState;
	uint8_t velocityTimer;
};

class Switches{
public:
	uint8_t offset;
	uint8_t numOfRegisters;
	uint8_t numOfStates;
	SWITCH_MODE mode;

	Switches(uint8_t nRegisters, uint8_t nStates, uint8_t srOffset, SWITCH_MODE sMode);
	~Switches(void);

	static void spi(void);
	void update(int subTick);

	void setLed(int n, PWM_MODE pwm);
	void setLed(int n, PWM_MODE pwm, PULSE_MODE pulse);
	void setLedAll(PWM_MODE pwm);
	void setLedAll(PULSE_MODE pulse);
	void setLedAll(PWM_MODE pwm, PULSE_MODE pulse);
	void setLedByte(int offset,uint8_t mask, PWM_MODE pwm, PULSE_MODE pulse);
	void setLedArray(int offset, int n, uint8_t* mask, PWM_MODE pwm, PULSE_MODE pulse);
	void setLedArray(int offset, int n, uint8_t* mask, bool inverted, PWM_MODE pwm, PULSE_MODE pulse);

	bool held(void);
	bool pressed(void);
	bool released(void);
	bool hasChanged(void);

	uint8_t returnPressed(uint8_t index);
	uint8_t returnReleased(uint8_t index);

	uint8_t firstHeld(void);
	uint8_t firstPressed(void);
	uint8_t firstChanged(void);

	bool isHeld(uint8_t sw);
	bool isPressed(uint8_t sw);
	bool isReleased(uint8_t sw);

	void currentState(uint8_t* state);
private:
	static std::vector<uint8_t> spi_in;
	static std::vector<uint8_t> spi_out;
	static uint8_t totalNumOfRegisters;
	static SPI_HandleTypeDef* spi_interface;

	std::vector<uint8_t> index;
	std::vector< std::vector<uint8_t> > state;
	std::vector<uint8_t> debouncedState;
	std::vector<uint8_t> changed;
	std::vector<uint8_t> holdTimer;

	std::vector<uint8_t> ledPWM;
	std::vector<uint8_t> ledPulse;
};



#endif /* SWITCHES_HPP_ */
