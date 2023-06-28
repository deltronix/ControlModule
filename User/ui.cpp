/*
 * ui.cpp
 *
 *  Created on: Dec 11, 2019
 *      Author: delta
 */
#include "ui.hpp"


UserInterface::UserInterface(void){
}
UserInterface::~UserInterface(void){}
void UserInterface::readSwitches(Switches& controlSwitches,Switches& stepSwitches, Scene& scene){
	if(controlSwitches.held()){
		switch(controlSwitches.firstHeld()){
		case CONTROL_SWITCH_SCENE:
			break;
		case CONTROL_SWITCH_PART:
			break;
		case CONTROL_SWITCH_LANE:
			scope = SCOPE_LANE;
			changeState(UI_STATE_OVERVIEW);
			controlSwitches.setLedByte(0, 0x0F, PWM_OFF, PULSE_MODE_OFF);
			controlSwitches.setLed(CONTROL_SWITCH_LANE, PWM_FULL, PULSE_MODE_OFF);
		}
	}
	else if(controlSwitches.hasChanged()){
		switch(controlSwitches.firstPressed()){
		case CONTROL_SWITCH_SCENE:
			changeScope(SCOPE_SCENE);
			controlSwitches.setLedByte(0, 0x0F, PWM_OFF, PULSE_MODE_OFF);
			controlSwitches.setLed(CONTROL_SWITCH_SCENE, PWM_FULL, PULSE_MODE_OFF);
			break;
		case CONTROL_SWITCH_PART:
			changeScope(SCOPE_PART);
			controlSwitches.setLedByte(0, 0x0F, PWM_OFF, PULSE_MODE_OFF);
			controlSwitches.setLed(CONTROL_SWITCH_PART, PWM_FULL, PULSE_MODE_OFF);
			break;
		case CONTROL_SWITCH_LANE:
			changeScope(SCOPE_LANE);
			controlSwitches.setLedByte(0, 0x0F, PWM_OFF, PULSE_MODE_OFF);
			controlSwitches.setLed(CONTROL_SWITCH_LANE, PWM_FULL, PULSE_MODE_OFF);
			break;
		case CONTROL_SWITCH_STEP:
			changeScope(SCOPE_STEP);
			controlSwitches.setLedByte(0, 0x0F, PWM_OFF, PULSE_MODE_OFF);
			controlSwitches.setLed(CONTROL_SWITCH_STEP, PWM_FULL, PULSE_MODE_OFF);
			break;
		case CONTROL_SWITCH_COPY:
			if(controlSwitches.isPressed(CONTROL_SWITCH_SHIFT)){
				changeState(UI_STATE_NOTIFY_SAVING);
			}
			else if(uiState == UI_STATE_COPY){
				scene.copy();
				changeState(previousUiState);
			}
			else{
				scene.selectAllInFocus(scope);
				changeState(UI_STATE_COPY);
			}

			break;
		case CONTROL_SWITCH_PASTE:
			if(controlSwitches.isPressed(CONTROL_SWITCH_SHIFT)){changeState(UI_STATE_NOTIFY_LOADING);			}
			else if(uiState == UI_STATE_PASTE){
				scene.paste();
				changeState(previousUiState);
			}
			else{
				changeState(UI_STATE_PASTE);
			}

			break;
		case CONTROL_SWITCH_START_STOP:
			if(scene.transportState == TRANSPORT_STATE_PLAY){
				scene.transportState = TRANSPORT_STATE_STOP;
				controlSwitches.setLed(8,PWM_FULL);
				controlSwitches.setLed(9,PWM_OFF);
			}
			else{
				scene.transportState = TRANSPORT_STATE_RESET;
				controlSwitches.setLed(8,PWM_OFF);
				controlSwitches.setLed(9,PWM_FULL);
			}
			break;

		default:
			break;
		}
	}
	if(stepSwitches.pressed()){
		switch(uiState){
		case UI_STATE_COPY:
			break;
		case UI_STATE_PASTE:
				scene.setPasteDestination(scope,stepSwitches.firstPressed());
			break;
		default:
			switch(scope){
			case SCOPE_SCENE:
				stepSwitches.setLedAll(PWM_OFF);
				break;
			case SCOPE_PART:
				scene.focusPart(stepSwitches.firstPressed());
				break;
			case SCOPE_LANE:
				if(stepSwitches.firstPressed() < 16){
					scene.focusLane(stepSwitches.firstPressed());
					changeScope(SCOPE_STEP);
				}
				else{
					scene.sceneData[scene.focusedScene].laneMute[scene.focusedPart] ^= (1 << (stepSwitches.firstPressed()-16));
				}


				break;
			case SCOPE_STEP:
				scene.toggleStepNoteOn(stepSwitches.firstPressed());
				scene.focusStep(stepSwitches.firstPressed());
				break;
			}
			break;
		}

	}

}
void UserInterface::readEncoders(Encoder& encoderA, Encoder& encoderB, Scene& scene){
	if(encoderA.update() | encoderB.update()){
		switch(scope){
		case SCOPE_STEP:
			scene.focusNextStepParameter(encoderA.readAndResetCounter());
			scene.incrementDecrementStepParameter(encoderB.readAndResetCounter());
			break;
		case SCOPE_LANE:
			scene.focusNextLaneParameter(encoderA.readAndResetCounter());
			scene.incrementDecrementLaneParameter(encoderB.readAndResetCounter());
			break;
		}

	}
}

void UserInterface::changeState(UI_STATE state){
	previousUiState = uiState;
	uiState = state;
}
void UserInterface::changeScope(SCOPE newScope){
	previousScope = scope;
	scope = newScope;
}
void UserInterface::setStepSwitchLeds(Switches& stepSwitches, Scene& scene){
	switch(scope){
	case SCOPE_SCENE:
		stepSwitches.setLedAll(PWM_OFF);
		break;
	case SCOPE_PART:
		stepSwitches.setLedAll(PWM_OFF);
		stepSwitches.setLed(scene.focusedPart,PWM_FULL);


		break;
	case SCOPE_LANE:
		stepSwitches.setLedAll(PWM_OFF);
		// stepSwitches.setLedArray(0,2,scene.gateBuffer,PWM_FULL,PULSE_MODE_OFF);
		stepSwitches.setLed(scene.focusedLane,PWM_FULL,PULSE_MODE_OFF);
		stepSwitches.setLedArray(2, 2,(uint8_t*) &scene.sceneData[scene.focusedScene].laneMute[scene.focusedPart], 0, PWM_FULL, PULSE_MODE_OFF);
		break;
	case SCOPE_STEP:
		uint8_t valueArray[stepSwitches.numOfRegisters];
		scene.getStepNoteOnMask(stepSwitches.numOfRegisters, valueArray);
		stepSwitches.setLedArray(0, stepSwitches.numOfRegisters, valueArray, 1, PWM_OFF, PULSE_MODE_OFF);
		stepSwitches.setLedArray(0, stepSwitches.numOfRegisters, valueArray, PWM_FULL, PULSE_MODE_OFF);
		stepSwitches.setLed(scene.getLaneActiveStep(),PWM_FULL,PULSE_MODE_16TH);
		break;
	}
}

void UserInterface::updateDisplay(Display& display, Scene& scene){

}


void UserInterface::updateDisplayAndMemoryDma(Display& display, Memory& memory, Scene& scene, Clock& clock, Switches& controlSwitches){
	if(display.doneWriting()){
		switch(uiState){
		case UI_STATE_NOTIFY_SAVING:
			display.clearBuffer();
			display.notification("SAVING", "DON'T TURN OFF");
			uiState = UI_STATE_SAVING;
			break;
		case UI_STATE_SAVING:
			break;
		case UI_STATE_NOTIFY_LOADING:
			display.clearBuffer();
			display.notification("LOADING", "DON'T TURN OFF");
			uiState = UI_STATE_LOADING;
			break;
		case UI_STATE_LOADING:
			break;
		case UI_STATE_SELECT:
			display.clearBuffer();
			display.notification("SELECT", "JUST DO IT");
		case UI_STATE_SETTINGS:
		case UI_STATE_OVERVIEW:
			char stringTx[20];
			sprintf(stringTx,"%02d:%c%d:%02d %s", 0, scene.focusedPart+65, scene.focusedLane+1, scene.focusedStep+1, SCOPE_STRING[scope]);
			display.clearBuffer();
			display.stringToBuffer(30,0,font_6x8,stringTx,0);
			switch(scope){
			case SCOPE_PART:
				break;
			case SCOPE_LANE:
				display.laneParameters(scene);
				break;
			case SCOPE_STEP:
				display.displayStepParameterGrid(scene.focusedStepParameter, scene.focusedStep, scene);
				break;
			case SCOPE_SCENE:
			sprintf(stringTx,"CLK: %d", clock.averagedPeriod);
			display.stringToBuffer(30,1,font_6x8,stringTx,0);
				break;
			default:
				break;
			}
			break;
			case UI_STATE_EDIT:

				break;
			case UI_STATE_COPY:
				display.clearBuffer();
				display.notification("COPY", "ADD TO SELECTION");
				break;
			case UI_STATE_PASTE:
				display.clearBuffer();
				display.notification("PASTE","SELECT TARGET");
		}
	}

	switch(memory.stateMachine()){
	case MEMORY_READY:
		if(display.writeDisplayBufferDMA()){
			if(uiState == UI_STATE_SAVING){
				memory.writeSceneDma(scene.activeScenePointer());
			}
			else if(uiState == UI_STATE_LOADING){
				memory.readSceneDma(scene.idleScenePointer());
			}
		}
		break;
	case MEMORY_WRITE_WAITING_FOR_ERASE:
	case MEMORY_WRITE_IN_PROGRESS:
	case MEMORY_WAITING_FOR_WRITE:
		break;
	case MEMORY_WRITE_DONE:
		uiState = UI_STATE_OVERVIEW;
		memory.completeTransfer();
		break;
	case MEMORY_READ_DONE:
		uiState = UI_STATE_OVERVIEW;
		scene.switchActiveScene();
		memory.completeTransfer();
		break;
	}
}
