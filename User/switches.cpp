/*
 * ui.cpp
 *
 *  Created on: Feb 4, 2019
 *      Author: delta
 */

#include "switches.hpp"
#include "stm32f4xx_hal.h"

uint8_t ui_spi_in[NUM_OF_REGISTERS];
uint8_t ui_spi_out[NUM_OF_REGISTERS];

std::vector<uint8_t> Switches::spi_in = {0,0,0,0,0,0};
std::vector<uint8_t> Switches::spi_out = {0,0,0,0,0,0};
uint8_t Switches::totalNumOfRegisters = 0;

Encoder::Encoder(uint8_t nStates,GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin_A, uint16_t GPIO_Pin_B, uint16_t GPIO_Pin_Sw){
	numOfStates = nStates;
	port = GPIO_Port;
	pinA = GPIO_Pin_A;
	pinB = GPIO_Pin_B;
	pinSw = GPIO_Pin_Sw;
	for(uint8_t i = 0; i < nStates; i++){
		fullState |= (1 << i);
	}
}
Encoder::~Encoder(){
	;
}
bool Encoder::update(void){
	// Remember the previous state of the pins in order to detect a state change
	bool oldStateA=debouncedStateA;
	bool oldStateB=debouncedStateB;
	bool oldStateSw=debouncedStateSw;

	// Read GPIO inputs and write to the debouncing buffer
	if(!(port->IDR & pinA))	{bufferA |= (1 << index);}
	else					{bufferA &= ~(1 << index);}
	
	if(!(port->IDR & pinB))	{bufferB |= (1 << index);}
	else					{bufferB &= ~(1 << index);}
	
	if(!(port->IDR & pinSw)){bufferSw |= (1 << index);}
	else					{bufferSw &= ~(1 << index);}
	
	// Increment or reset the position in the debouncing buffer
	if(++index >= numOfStates){index = 0;}
		
	// Check if the buffer is full or empty (e.g. fullState is 00001111 when numOfStates is 4).
	if(bufferA == fullState)	{debouncedStateA = 1;}
	else if(bufferA == 0)		{debouncedStateA = 0;}
	
	if(bufferB == fullState)	{debouncedStateB = 1;}
	else if(bufferB == 0)		{debouncedStateB = 0;}
	
	if(bufferSw == fullState)	{debouncedStateSw = 1;}
	else if(bufferSw == 0)		{debouncedStateSw = 0;}

	// Update switch state
	if(debouncedStateSw!=oldStateSw){
		switchChanged = true;
	}
	else{
		switchChanged = false;
	}

	
	encoderChanged = false;
	
	// Check if pin A's state is different from pin B
	if(debouncedStateA ^ debouncedStateB){
		// If pin A has changed the encoder was turned clockwise
		if(debouncedStateA != oldStateA){
			encoderChanged = true;
			counter++;
		}
		// If pin B has changed the encoder was turned counter clockwise
		else if(debouncedStateB != oldStateB){
			encoderChanged = true;
			counter--;
		}
	}
	return encoderChanged | switchChanged;
}
bool Encoder::hasChanged(void){
	return switchChanged | encoderChanged;
}
bool Encoder::wasPressed(void){
	return switchChanged & debouncedStateSw;
}
bool Encoder::wasReleased(void){
	return switchChanged & !(debouncedStateSw);
}
int16_t Encoder::readAndResetCounter(void){
	int16_t counterBuffer = counter;
	counter = 0;
	return counterBuffer;
}

// Switch debouncer
Switches::Switches(uint8_t nRegisters, uint8_t nStates, uint8_t srOffset, SWITCH_MODE sMode){
	numOfRegisters = nRegisters;
	numOfStates = nStates;
	offset = srOffset;
	mode = sMode;

	totalNumOfRegisters += nRegisters;

	Switches::spi_out.resize(totalNumOfRegisters);
	Switches::spi_in.resize(totalNumOfRegisters);

	state.resize(numOfRegisters, std::vector<uint8_t>(numOfStates));
	debouncedState.resize(numOfRegisters);
	changed.resize(numOfRegisters);
	index.resize(numOfRegisters);

	holdTimer.resize(numOfRegisters*8);
	ledPWM.resize(numOfRegisters*8, 0);
	ledPulse.resize(numOfRegisters*8, 0);

}
Switches::~Switches(void){
	;
}
void Switches::spi(void){
	HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin,GPIO_PIN_SET);
	HAL_SPI_TransmitReceive(spi_interface, Switches::spi_out.data(), Switches::spi_in.data(), totalNumOfRegisters, 10);
	// Transfer shift register contents to storage register
	HAL_Delay(1);
	HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin,GPIO_PIN_SET);
}
void Switches::update(int subTick){
	uint8_t lastDebouncedState[numOfRegisters];
	uint8_t ledMask[numOfRegisters];
	uint8_t i, x, j;
	static uint8_t PWMcounter;
	int shift;
	uint8_t pulse = subTick/24;

	if(++PWMcounter >= 64){
		PWMcounter = 1;
	}

	for(x = 0; x < numOfRegisters; x++){
		// Dim and pulse leds
		for(j = 0; j<8; j++){
			shift = (x*8)+j;

			if(ledPulse[shift]){
				if(!(pulse%ledPulse[shift])){
					if(PWMcounter<ledPWM[shift]){

						ledMask[x] |= (1 << j);
					}
					else{
						ledMask[x] &= ~(1 << j);

					}
				}
				else{
					// ledMask[x] |= (1 << j);
					ledMask[x] &= ~(1 << j);
				}
			}
			else if(ledPWM[shift]){
				if(PWMcounter<ledPWM[shift]){
					ledMask[x] |= (1 << j);
				}
				else{
					//ledMask[x] |= (1 << j);
					ledMask[x] &= ~(1 << j);
				}
			}
			else{
				//ledMask[x] |= (1 << j);
				ledMask[x] &= ~(1 << j);
			}

		}

		// Read and write from SPI buffers
		// THIS SHOULD BE MADE MORE GENERIC!

		if(mode == SWITCH_MODE_STEP){
			Switches::spi_out[1] = (ledMask[0]&0x0F) | (ledMask[1]<<4);
			Switches::spi_out[3] = (ledMask[1]&0xF0) | (ledMask[0]>>4);

			Switches::spi_out[0] = (ledMask[2]&0x0F) | (ledMask[3]<<4);
			Switches::spi_out[2] = (ledMask[3]&0xF0) | (ledMask[2]>>4);

			if(!(x%2)){
				int y = ((x)/2)+(2)+offset;
				int z = ((x+1)/2)+offset;
				state[x][index[x]] = (Switches::spi_in[y]&0x0F) | (Switches::spi_in[z]<<4);

			}
			else{
				int y = ((x-1)/2)+(2)+offset;
				int z = ((x)/2)+offset;
				state[x][index[x]] = (Switches::spi_in[y]>>4) | (Switches::spi_in[z]&0xF0);
			}
		}
		else{
			Switches::spi_out[totalNumOfRegisters-(1+offset+x)] = ledMask[x];
			state[x][index[x]] = Switches::spi_in[x+offset];
		}



		lastDebouncedState[x] = debouncedState[x];
		debouncedState[x] = 0xFF;

		// Debounce switches
		for(i = 0; i < numOfStates; i++){
			debouncedState[x] &= ~state[x][i];
		}

		index[x]++;
		if(index[x] >= numOfStates){
			index[x] = 0;
		}

		changed[x] = debouncedState[x] ^ lastDebouncedState[x];

		// Add to hold time or reset
		for(uint8_t i = 0; i < 8; i++){
			if(debouncedState[x] & (1 << i)){
				//Prevent overflow
				if(!(++holdTimer[(x<<3)+i])){
					holdTimer[(x<<3)+i] = 255;
				}
			}
			else{
				holdTimer[(x<<3)+i] = 0;
			}


		}
	}
}

// ---------------------------- SWITCH FUNCTIONS  ----------------------------

bool Switches::hasChanged(void){
	uint8_t change = 0;
	for(uint8_t i = 0; i < numOfRegisters; i++){
		change |= changed[i];
	}
	if(change)
		return true;
	else
		return false;
}

uint8_t Switches::firstChanged(void){
	for(uint8_t i = 0; i < numOfRegisters; i++){
		for(int j = 0; j < 8; j++){
			if((changed[i] & (1 << j))){
				return (i*8)+j;
			}
		}
	}
	return -1;
}
uint8_t Switches::firstPressed(void){
	for(uint8_t i = 0; i < numOfRegisters; i++){
		for(int j = 0; j < 8; j++){
			if((changed[i] & debouncedState[i] & (1 << j))){
				return (i*8)+j;
			}
		}
	}
	return -1;
}
uint8_t Switches::firstHeld(void){
	for(uint8_t i = 0; i < numOfRegisters*8; i++){
		if(holdTimer[i] > 64){
			return i;
		}
	}
}

uint8_t Switches::returnPressed(uint8_t index){
	return changed[index] & debouncedState[index];
}


bool Switches::held(void){
	for(uint8_t i = 0; i < numOfRegisters*8; i++){
		if(holdTimer[i] > 64){
			return true;
		}
	}
	return false;
}
bool Switches::pressed(void){
	uint8_t pressed = 0;
	for(uint8_t i = 0; i < numOfRegisters; i++){
		pressed |= changed[i] & debouncedState[i];
	}
	if(pressed != 0)
		return true;
	else
		return false;
}
bool Switches::released(void){
	uint8_t released = 0;
	for(uint8_t i = 0; i < numOfRegisters; i++){
		released |= changed[i] & ~(debouncedState[i]);
	}
	if(released)
		return true;
	else
		return false;
}

bool Switches::isHeld(uint8_t sw){
	if(holdTimer[sw] > 64){
		return true;
	}
	return false;
}
bool Switches::isPressed(uint8_t sw){
	if(debouncedState[sw / 8] & (1 << (sw % 8))){
		return true;
	}
	return false;
}
bool Switches::isReleased(uint8_t sw){
	if(~(debouncedState[sw / 8]) & (1 << (sw % 8))){
		return true;
	}
	return false;
}
uint8_t Switches::returnReleased(uint8_t index){
	return changed[index] & ~(debouncedState[index]);
}



void Switches::currentState(uint8_t* state){
	for(uint8_t i = 0; i < numOfRegisters; i++){
		*(state+i) = debouncedState[i];
	}
}
void Switches::setLed(int n, PWM_MODE pwm){
	ledPWM[n] = pwm;
}
void Switches::setLed(int n, PWM_MODE pwm, PULSE_MODE pulse){
	ledPWM[n] = pwm;
	ledPulse[n] = pulse;
}

void Switches::setLedArray(int offset, int n, uint8_t* mask, PWM_MODE pwm, PULSE_MODE pulse){
	for(int i = offset; i < offset+n; i++){
		for(int j = 0; j < 8; j++){
			if(mask[i] & (1 << j)){
				int step = i*8+j;
				ledPulse[step] = pulse;
				ledPWM[step] = pwm;
			}
		}
	}
}
void Switches::setLedArray(int offset, int n, uint8_t* mask, bool inverted, PWM_MODE pwm, PULSE_MODE pulse){
	if(inverted){
		for(int i = 0; i < n; i++){
			for(int j = 0; j < 8; j++){
				if(~(mask[i]) & (1 << j)){
					int step = ((i+offset)*8)+j;
					ledPulse[step] = pulse;
					ledPWM[step] = pwm;
				}
			}
		}
	}
	else{
		for(int i = 0; i < n; i++){
			for(int j = 0; j < 8; j++){
				if(mask[i] & (1 << j)){
					int step = ((i+offset)*8)+j;
					ledPulse[step] = pulse;
					ledPWM[step] = pwm;
				}
			}
		}
	}
}
void Switches::setLedByte(int offset,uint8_t mask, PWM_MODE pwm, PULSE_MODE pulse){
	for(int j = 0; j < 8; j++){
		if(mask & (1 << j)){
			int step = (offset)*8+j;
			ledPulse[step] = pulse;
			ledPWM[step] = pwm;
		}
	}
}
void Switches::setLedAll(PWM_MODE pwm){
	for(int i = 0; i < numOfRegisters*8; i++){
		ledPWM[i] = pwm;
		ledPulse[i] = PULSE_MODE_OFF;
	}
}
void Switches::setLedAll(PULSE_MODE pulse){
	for(int i = 0; i < numOfRegisters*8; i++){
		ledPulse[i] = pulse;
		ledPWM[i] = PWM_FULL;
	}
}
void Switches::setLedAll(PWM_MODE pwm, PULSE_MODE pulse){
	for(int i = 0; i < numOfRegisters*8; i++){
		ledPWM[i] = pwm;
		ledPulse[i] = pulse;
	}
}
