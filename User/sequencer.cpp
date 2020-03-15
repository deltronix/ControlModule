/*
 * sequencer.cpp
 *
 *  Created on: Feb 4, 2019
 *      Author: delta
 */
#include <switches.hpp>
#include <vector>
#include "sequencer.hpp"

Sequencer::Sequencer(){}
void Sequencer::readControlSwitches(Switches& controlSwitches, Switches& stepSwitches){
	// Check whether a function or step switch has been pressed

	uint16_t step = 0;
	if(controlSwitches.pressed()){
		switch(controlSwitches.returnPressed(0)){
		case PLAY:
			if(transportState == TRANSPORT_STATE_STOP){
				transportState = TRANSPORT_STATE_RESET;
			}
			else{
				transportState = TRANSPORT_STATE_STOP;
			}
			break;
		case PAUSE:
			break;
		case COPY:
			break;
		case PASTE:
			break;
		case SELECT_BLOCK:
			controlSwitches.setLedByte(0,SELECT_BLOCK|SELECT_LANE|SELECT_NOTE|SELECT_SONG,PWM_OFF, PULSE_MODE_OFF);
			controlSwitches.setLedByte(0,SELECT_BLOCK,PWM_HALF,PULSE_MODE_4TH);
			stepSwitches.setLedAll(PWM_OFF,PULSE_MODE_OFF);
			stepSwitches.setLed(playingBlock, PWM_HALF, PULSE_MODE_4TH);
			stepSwitchMode = STEPSWITCH_SELECT_BLOCK;
			break;
		case SELECT_LANE:
			controlSwitches.setLedByte(0,SELECT_BLOCK|SELECT_LANE|SELECT_NOTE|SELECT_SONG,PWM_OFF,PULSE_MODE_OFF);
			controlSwitches.setLedByte(0,SELECT_LANE,PWM_HALF,PULSE_MODE_8TH);
			stepSwitches.setLedAll(PWM_OFF,PULSE_MODE_OFF);
			stepSwitches.setLed(selectedLane, PWM_HALF, PULSE_MODE_8TH);
			stepSwitchMode = STEPSWITCH_SELECT_LANE;
			break;
		case SELECT_NOTE:
			controlSwitches.setLedByte(0,SELECT_BLOCK|SELECT_LANE|SELECT_NOTE|SELECT_SONG,PWM_OFF,PULSE_MODE_OFF);
			controlSwitches.setLedByte(0,SELECT_NOTE,PWM_HALF,PULSE_MODE_16TH);
			stepSwitches.setLedAll(PWM_OFF,PULSE_MODE_OFF);
			stepSwitches.setLed(selectedStep, PWM_HALF, PULSE_MODE_16TH);
			stepSwitchMode = STEPSWITCH_SELECT_NOTE;
			break;
		case SELECT_SONG:
			stepSwitches.setLedAll(PWM_OFF,PULSE_MODE_OFF);
			stepSwitches.setLed(selectedSong, PWM_HALF, PULSE_MODE_2ND);
			stepSwitchMode = STEPSWITCH_SELECT_SONG;
			break;
		default:
			break;
		}
	}
	switch (stepSwitchMode){
	case STEPSWITCH_SELECT_BLOCK:
		// stepSwitches.setLedAll(PWM_OFF);
		// stepSwitches.setLed(selectedBlock, PWM_FULL, PULSE_MODE_4TH);
		if(stepSwitches.pressed()){
			selectedBlock = stepSwitches.firstChanged();
			stepSwitches.setLedAll(PWM_OFF,PULSE_MODE_OFF);
			stepSwitchMode = STEPSWITCH_EDIT_BLOCK;
		}
		break;
	case STEPSWITCH_SELECT_LANE:
		// stepSwitches.setLedAll(PWM_OFF);
		// stepSwitches.setLed(selectedBlock, PWM_FULL, PULSE_MODE_8TH);
		if(stepSwitches.pressed()){
			selectedLane = stepSwitches.firstChanged();
			stepSwitches.setLedAll(PWM_OFF,PULSE_MODE_OFF);
			stepSwitchMode = STEPSWITCH_EDIT_BLOCK;
		}

		break;
	case STEPSWITCH_SELECT_NOTE:
		stepSwitches.setLedAll(PWM_OFF,PULSE_MODE_OFF);
		stepSwitches.setLed(selectedStep, PWM_FULL, PULSE_MODE_16TH);
		if(stepSwitches.released()){
			selectedStep = stepSwitches.firstChanged();
			stepSwitches.setLedAll(PWM_OFF,PULSE_MODE_OFF);
			stepSwitchMode = STEPSWITCH_EDIT_BLOCK;
			break;
		}
		break;

	case STEPSWITCH_SELECT_SONG:
		stepSwitches.setLedAll(PWM_OFF,PULSE_MODE_OFF);
		stepSwitches.setLed(selectedSong, PWM_FULL);
		break;
	case STEPSWITCH_EDIT_BLOCK:
		for(int i=0;i < stepSwitches.numOfRegisters;i++){
			for(int j=0;j<8;j++){
				step = (i*8)+j;
				// Set note
				if(stepSwitches.returnPressed(i) & (1 << j)){
					song[selectedSong].subSteps[selectedLane][selectedBlock][step] ^= (1 << 0);
				}
				// Set leds
				if(step == song[selectedSong].activeStep[selectedLane]){
					if(song[selectedSong].subSteps[selectedLane][selectedBlock][step]){
						stepSwitches.setLed(step, PWM_DIM);
					}
					else{
						stepSwitches.setLed(step, PWM_HALF);
					}
				}
				else if(song[selectedSong].subSteps[selectedLane][selectedBlock][step]){
					if(step == selectedStep){
						stepSwitches.setLed(selectedStep,PWM_HALF,PULSE_MODE_16TH);
					}
					else{
						stepSwitches.setLed(step,PWM_HALF);
					}
				}
				else{
					if(step == selectedStep){
						stepSwitches.setLed(selectedStep,PWM_DIM,PULSE_MODE_OFF);
					}
					else{
						stepSwitches.setLed(step,PWM_OFF);
					}
				}
			}
		}
		break;
	default:
		break;
	}
}
void Sequencer::writeTrigPattern(void){
	uint8_t step;

	static uint8_t previousUartTrigger;
	char uartBuffer[3];

	if(transportState == TRANSPORT_STATE_PLAY){
		uint8_t trigger = 0;
		for(int i = 0; i < 2; i++){
			step = song[playingSong].activeStep[i];

			if((song[playingSong].subSteps[i][playingBlock][step]) & (1 <<  song[playingSong].activeSubStep)){
				HAL_GPIO_WritePin(GPIOB, (1 << i), GPIO_PIN_SET);
				trigger |= (1 << i);
			}
			else{
				HAL_GPIO_WritePin(GPIOB, (1 << i), GPIO_PIN_RESET);
			}
		}
		char uartTrigger = 0;
		for(int i = 0; i < 8; i++){
			step = song[playingSong].activeStep[i];
			if((song[playingSong].subSteps[i][playingBlock][step]) & (1 <<  song[playingSong].activeSubStep)){
				uartTrigger |= (1 << i);

			}
		}
		if(uartTrigger != previousUartTrigger){
			previousUartTrigger = uartTrigger;
			//sprintf(uartBuffer,"%c%c",'T',uartTrigger);
			HAL_UART_Transmit(&huart2,(uint8_t*)uartBuffer,2,10);
		}
	}
}



Sequencer::~Sequencer(void){
}




