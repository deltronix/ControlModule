/*
 * scene.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: delta
 */


#include "scene.hpp"
#include "scales.hpp"
#include <iostream>
#include <stm32f4xx_hal.h>



Scene::Scene(uint16_t beatClockTicks){
	beatTicks = beatClockTicks;
	stepTicks = beatClockTicks/nSteps;

}

Scene::~Scene(){}

uint32_t Scene::getMainClock(void){
	return sceneData[activeScene].mainClock;
}

SceneData& Scene::activeScenePointer(void){
	return sceneData[activeScene];
}
SceneData& Scene::idleScenePointer(void){
	if(activeScene == 0){
		return sceneData[1];
	}
	else{
		return sceneData[0];
	}
}
uint8_t Scene::switchActiveScene(void){
	if(activeScene == 0){
		activeScene = 1;
		focusedScene = 1;
	}
	else{
		activeScene = 0;
		focusedScene = 0;
	}
	return activeScene;
}

STEP_PARAMETER Scene::focusNextStepParameter(int16_t n){
	while(n){
		if(n < 0){
			switch(focusedStepParameter){
			case STEP_PARAMETER_NOTE_ON:
				focusedStepParameter = STEP_PARAMETER_GLIDE;
				break;
			case STEP_PARAMETER_REPEAT:
				focusedStepParameter = STEP_PARAMETER_NOTE_ON;
				break;
			case STEP_PARAMETER_PULSE_WIDTH:
				focusedStepParameter = STEP_PARAMETER_REPEAT;
				break;
			case STEP_PARAMETER_LENGTH:
				focusedStepParameter = STEP_PARAMETER_PULSE_WIDTH;
				break;
			case STEP_PARAMETER_PROBABILITY:
				focusedStepParameter = STEP_PARAMETER_LENGTH;
				break;
			case STEP_PARAMETER_INDEX:
				focusedStepParameter = STEP_PARAMETER_PROBABILITY;
				break;
			case STEP_PARAMETER_GLIDE:
				focusedStepParameter = STEP_PARAMETER_INDEX;
				break;
			default:
				break;
			}
			n++;
		}
		else if(n > 0){
			switch(focusedStepParameter){
			case STEP_PARAMETER_NOTE_ON:
				focusedStepParameter = STEP_PARAMETER_REPEAT;
				break;
			case STEP_PARAMETER_REPEAT:
				focusedStepParameter = STEP_PARAMETER_PULSE_WIDTH;
				break;
			case STEP_PARAMETER_PULSE_WIDTH:
				focusedStepParameter = STEP_PARAMETER_LENGTH;
				break;
			case STEP_PARAMETER_LENGTH:
				focusedStepParameter = STEP_PARAMETER_PROBABILITY;
				break;
			case STEP_PARAMETER_PROBABILITY:
				focusedStepParameter = STEP_PARAMETER_INDEX;
				break;
			case STEP_PARAMETER_INDEX:
				focusedStepParameter = STEP_PARAMETER_GLIDE;
				break;
			case STEP_PARAMETER_GLIDE:
				focusedStepParameter = STEP_PARAMETER_NOTE_ON;
				break;
			default:
				break;
			}
			n--;
		}
		else{
			return focusedStepParameter;
		}
	}
	return focusedStepParameter;
}
LANE_PARAMETER Scene::focusNextLaneParameter(int16_t n){
	if((focusedLaneParameter + n) >= SIZE_OF_LANE_PARAMETERS){
		focusedLaneParameter = static_cast<LANE_PARAMETER>(0);
	}
	else if((focusedLaneParameter + n) < 0){
		focusedLaneParameter = static_cast<LANE_PARAMETER>(SIZE_OF_LANE_PARAMETERS - 1);
	}
	else{
		focusedLaneParameter = static_cast<LANE_PARAMETER>(focusedLaneParameter + n);
	}
	return focusedLaneParameter;
}
const char*	Scene::stepParameterString(){
	switch(focusedStepParameter){
	case STEP_PARAMETER_NOTE_ON:
		return "NOTE";
		break;
	case STEP_PARAMETER_REPEAT:
		return "REP ";
		break;
	case STEP_PARAMETER_PULSE_WIDTH:
		return "PW  ";
		break;
	case STEP_PARAMETER_LENGTH:
		return "LEN ";
		break;
	case STEP_PARAMETER_PROBABILITY:
		return "PROB";
		break;
	case STEP_PARAMETER_INDEX:
		return "INDX";
		break;
	case STEP_PARAMETER_GLIDE:
		return "GLD ";
		break;
	default:
		break;
	}
}

void Scene::focusPart(uint8_t part){
	if(part < nParts){
		focusedPart = part;
	}
	else{
		focusedPart = nParts-1;
	}

}
void Scene::focusLane(uint8_t lane){
	if(lane < nLanes){
		focusedLane = lane;
	}
	else{
		focusedLane = nLanes-1;
	}
}
void Scene::focusStep(uint8_t step){
	if(step < nSteps){
		focusedStep = step;

	}
	else{
		focusedStep = nSteps-1;
	}
}


uint8_t Scene::getStepParameterArray(STEP_PARAMETER param, uint8_t fromPart, uint8_t fromLane, uint8_t* valueArray){
	int i;
	for(i = 0; i < nSteps; i++){
		valueArray[i] = getStepParameter(param, fromPart, fromLane, i);
	}
	return i;
}
uint8_t Scene::getStepParameterArray(STEP_PARAMETER param, uint8_t* valueArray){
	int i;
	for(i = 0; i < nSteps; i++){
		valueArray[i] = getStepParameter(param, i);
	}
	return i;
}

uint8_t Scene::getStepNoteOnMask(uint8_t fromPart, uint8_t fromLane, uint8_t n, uint8_t* valueArray){
	int i;
	for(i = 0; i < n; i++){
		for(int j = 0; j < 8; j++){
			if(getStepParameter(STEP_PARAMETER_NOTE_ON, fromPart, fromLane, (i*8)+j)){
				valueArray[i] |= (1 << j);
			}
			else{
				valueArray[i] &= ~(1 << j);
			}

		}
	}
	return i;
}
uint8_t Scene::getStepNoteOnMask(uint8_t n, uint8_t* valueArray){
	int i;
	for(i = 0; i < n; i++){
		for(int j = 0; j < 8; j++){
			if(getStepParameter(STEP_PARAMETER_NOTE_ON, focusedPart, focusedLane, (i*8)+j)){
				valueArray[i] |= (1 << j);
			}
			else{
				valueArray[i] &= ~(1 << j);
			}

		}
	}
	return i;
}

void Scene::toggleStepNoteOn(uint8_t step){
	sceneData[activeScene].lane[focusedPart][focusedLane].step[step] ^=  STEP_PARAMETER_MASK_NOTE_ON;
}

void Scene::incrementDecrementStepParameter(int16_t n){
	if(focusedStepParameter == STEP_PARAMETER_INDEX){
		setStepParameter(focusedStepParameter, focusedStep,
				getStepParameter(focusedStepParameter, focusedStep) + n - sceneData[activeScene].lane[focusedPart][focusedLane].indexOffset);
	}
	else{
		setStepParameter(focusedStepParameter, focusedStep,
				getStepParameter(focusedStepParameter, focusedStep) + n);
	}
}
void Scene::incrementDecrementLaneParameter(int n){
	setLaneParameter(focusedLaneParameter, focusedPart, focusedLane,
			getLaneParameter(focusedLaneParameter, focusedPart, focusedLane) + n);
}

// Set a param for a given step in the selected sceneData[activeScene].lane and lane
void Scene::setStepParameter(STEP_PARAMETER param, uint8_t step, int8_t value){
	switch(param){
	case STEP_PARAMETER_NOTE_ON:
		sceneData[activeScene].lane[focusedPart][focusedLane].step[step] &=  ~(STEP_PARAMETER_MASK_NOTE_ON);
		sceneData[activeScene].lane[focusedPart][focusedLane].step[step] |= (value<<param) & STEP_PARAMETER_MASK_NOTE_ON;
		break;
	case STEP_PARAMETER_REPEAT:
		sceneData[activeScene].lane[focusedPart][focusedLane].step[step] &=  ~(STEP_PARAMETER_MASK_REPEAT);
		sceneData[activeScene].lane[focusedPart][focusedLane].step[step] |= (value<<param) & STEP_PARAMETER_MASK_REPEAT;
		break;
	case STEP_PARAMETER_PULSE_WIDTH:
		sceneData[activeScene].lane[focusedPart][focusedLane].step[step] &=  ~(STEP_PARAMETER_MASK_PULSE_WIDTH);
		sceneData[activeScene].lane[focusedPart][focusedLane].step[step] |= (value<<param) & STEP_PARAMETER_MASK_PULSE_WIDTH;
		break;
	case STEP_PARAMETER_LENGTH:
		sceneData[activeScene].lane[focusedPart][focusedLane].step[step] &=  ~(STEP_PARAMETER_MASK_LENGTH);
		sceneData[activeScene].lane[focusedPart][focusedLane].step[step] |= (value<<param) & STEP_PARAMETER_MASK_LENGTH;
		break;
	case STEP_PARAMETER_PROBABILITY:
		sceneData[activeScene].lane[focusedPart][focusedLane].step[step] &=  ~(STEP_PARAMETER_MASK_PROBABILITY);
		sceneData[activeScene].lane[focusedPart][focusedLane].step[step] |= (value<<param) & STEP_PARAMETER_MASK_PROBABILITY;
		break;
	case STEP_PARAMETER_INDEX:
		setStepIndex(focusedPart, focusedLane, step, value);
		break;

	}
}
void Scene::setStepParameter(STEP_PARAMETER param, uint8_t fromLane, uint8_t step, int8_t value){
	switch(param){
	case STEP_PARAMETER_NOTE_ON:
		sceneData[activeScene].lane[focusedPart][fromLane].step[step] &=  ~(STEP_PARAMETER_MASK_NOTE_ON);
		sceneData[activeScene].lane[focusedPart][fromLane].step[step] |= (value<<param) & STEP_PARAMETER_MASK_NOTE_ON;
		break;
	case STEP_PARAMETER_REPEAT:
		sceneData[activeScene].lane[focusedPart][fromLane].step[step] &=  ~(STEP_PARAMETER_MASK_REPEAT);
		sceneData[activeScene].lane[focusedPart][fromLane].step[step] |= (value<<param) & STEP_PARAMETER_MASK_REPEAT;
		break;
	case STEP_PARAMETER_PULSE_WIDTH:
		sceneData[activeScene].lane[focusedPart][fromLane].step[step] &=  ~(STEP_PARAMETER_MASK_PULSE_WIDTH);
		sceneData[activeScene].lane[focusedPart][fromLane].step[step] |= (value<<param) & STEP_PARAMETER_MASK_PULSE_WIDTH;
		break;
	case STEP_PARAMETER_LENGTH:
		sceneData[activeScene].lane[focusedPart][fromLane].step[step] &=  ~(STEP_PARAMETER_MASK_LENGTH);
		sceneData[activeScene].lane[focusedPart][fromLane].step[step] |= (value<<param) & STEP_PARAMETER_MASK_LENGTH;
		break;
	case STEP_PARAMETER_PROBABILITY:
		sceneData[activeScene].lane[focusedPart][fromLane].step[step] &=  ~(STEP_PARAMETER_MASK_PROBABILITY);
		sceneData[activeScene].lane[focusedPart][fromLane].step[step] |= (value<<param) & STEP_PARAMETER_MASK_PROBABILITY;
		break;
	case STEP_PARAMETER_INDEX:
		setStepIndex(focusedPart, fromLane, step, value);
		break;
	}
}

uint8_t Scene::getStepParameter(STEP_PARAMETER param, uint8_t step){
	switch(param){
	case STEP_PARAMETER_NOTE_ON:
		return (sceneData[activeScene].lane[focusedPart][focusedLane].step[step] & STEP_PARAMETER_MASK_NOTE_ON);
		break;
	case STEP_PARAMETER_REPEAT:
		return (sceneData[activeScene].lane[focusedPart][focusedLane].step[step] & STEP_PARAMETER_MASK_REPEAT) >> STEP_PARAMETER_REPEAT;
		break;
	case STEP_PARAMETER_PULSE_WIDTH:
		return (sceneData[activeScene].lane[focusedPart][focusedLane].step[step] & STEP_PARAMETER_MASK_PULSE_WIDTH) >> STEP_PARAMETER_PULSE_WIDTH;
		break;
	case STEP_PARAMETER_LENGTH:
		return (sceneData[activeScene].lane[focusedPart][focusedLane].step[step] & STEP_PARAMETER_MASK_LENGTH) >> STEP_PARAMETER_LENGTH;
		break;
	case STEP_PARAMETER_PROBABILITY:
		return (sceneData[activeScene].lane[focusedPart][focusedLane].step[step] & STEP_PARAMETER_MASK_PROBABILITY) >> STEP_PARAMETER_PROBABILITY;
		break;
	case STEP_PARAMETER_INDEX:
		return getStepIndex(focusedPart, focusedLane, step) + sceneData[activeScene].lane[focusedPart][focusedLane].indexOffset;
		break;
	default:
		return 0;
	}
}
uint8_t Scene::getStepParameter(STEP_PARAMETER param, uint8_t fromPart, uint8_t fromLane, uint8_t step){
	switch(param){
	case STEP_PARAMETER_NOTE_ON:
		return (sceneData[activeScene].lane[fromPart][fromLane].step[step] & STEP_PARAMETER_MASK_NOTE_ON);
		break;
	case STEP_PARAMETER_REPEAT:
		return (sceneData[activeScene].lane[fromPart][fromLane].step[step] & STEP_PARAMETER_MASK_REPEAT) >> STEP_PARAMETER_REPEAT;
		break;
	case STEP_PARAMETER_PULSE_WIDTH:
		return (sceneData[activeScene].lane[fromPart][fromLane].step[step] & STEP_PARAMETER_MASK_PULSE_WIDTH) >> STEP_PARAMETER_PULSE_WIDTH;
		break;
	case STEP_PARAMETER_LENGTH:
		return (sceneData[activeScene].lane[fromPart][fromLane].step[step] & STEP_PARAMETER_MASK_LENGTH) >> STEP_PARAMETER_LENGTH;
		break;
	case STEP_PARAMETER_PROBABILITY:
		return (sceneData[activeScene].lane[fromPart][fromLane].step[step] & STEP_PARAMETER_MASK_PROBABILITY) >> STEP_PARAMETER_PROBABILITY;
		break;
	case STEP_PARAMETER_INDEX:
		return getStepIndex(fromPart, fromLane, step) + sceneData[activeScene].lane[fromPart][fromLane].indexOffset;
		break;
	}
	return 0;
}
// Update the digital output buffer according to the clock phase, return true when a bit has updated

void Scene::setLaneParameter(LANE_PARAMETER param, uint8_t fromPart, uint8_t fromLane, uint8_t value){
	switch(param){
	case LANE_PARAMETER_END_STEP:
		sceneData[activeScene].lane[fromPart][fromLane].endStep = (value & 0x1F);
		break;
	case LANE_PARAMETER_DIVISION:
		sceneData[activeScene].lane[fromPart][fromLane].division = (value & 0x1F);
		break;
	case LANE_PARAMETER_INDEX_OFFSET:
		sceneData[activeScene].lane[fromPart][fromLane].indexOffset = (value & 0x7F);
		break;
	case LANE_PARAMETER_SCALE:
		break;
	case LANE_PARAMETER_CV_MODE:
		break;
	case LANE_PARAMETER_MIDI_CHAN:
		sceneData[activeScene].lane[fromPart][fromLane].midiChan = (value & 0x1F);
		break;
	case LANE_PARAMETER_MUTE:
		sceneData[activeScene].laneMute[fromPart] = ((value & 0x01) << fromLane);
		break;
	}
}
uint8_t Scene::getLaneParameter(LANE_PARAMETER param, uint8_t fromPart, uint8_t fromLane){
	switch(param){
	case LANE_PARAMETER_END_STEP:
		return sceneData[activeScene].lane[fromPart][fromLane].endStep;
		break;
	case LANE_PARAMETER_DIVISION:
		return sceneData[activeScene].lane[fromPart][fromLane].division;
		break;
	case LANE_PARAMETER_INDEX_OFFSET:
		return sceneData[activeScene].lane[fromPart][fromLane].indexOffset;
		break;
	case LANE_PARAMETER_SCALE:
		return 0;
		break;
	case LANE_PARAMETER_CV_MODE:
		return 0;
		break;
	case LANE_PARAMETER_MIDI_CHAN:
		return sceneData[activeScene].lane[fromPart][fromLane].midiChan;
		break;
	case LANE_PARAMETER_MUTE:
		return (sceneData[activeScene].laneMute[fromPart] >> fromLane) & 0x01;
		break;
	}
}


bool Scene::updateGateBuffer(uint16_t clock){

	if((transportState==TRANSPORT_STATE_PLAY)){
		for(int j = 0; j < 2; j++){
			previousgateBuffer[j] = gateBuffer[j];

			for(int i = 0; i < 8; i++){
				uint8_t lane = i+(j*8);
				uint8_t activeStep = sceneData[activeScene].lane[activePart][lane].activeStep;
				uint16_t step = sceneData[activeScene].lane[activePart][lane].step[activeStep];
				uint8_t length = 1;
				if(step & STEP_PARAMETER_MASK_LENGTH){
					length += getStepParameter(STEP_PARAMETER_LENGTH,activePart,lane,activeStep);
				}
				length+= getLaneParameter(LANE_PARAMETER_DIVISION, activePart, lane);

				if(!(step & 0x01)){
					gateBuffer[j] &= ~(1<<i);
				}
				else if(!(sceneData[activeScene].lane[activePart][lane].laneClock)){
					gateBuffer[j] |= (1<<i);
				}
				else if(step & (STEP_PARAMETER_MASK_REPEAT | STEP_PARAMETER_MASK_PULSE_WIDTH | STEP_PARAMETER_MASK_LENGTH)){
					uint8_t repeat = 1;
					uint8_t pulseWidth = 1;
					uint16_t repeatTicks = stepTicks*length;

					if(step & STEP_PARAMETER_MASK_REPEAT){
						repeat += getStepParameter(STEP_PARAMETER_REPEAT,activePart,lane,activeStep);
						repeatTicks = length*(stepTicks+(stepTicks%repeat))/repeat;
					}
					if(step & STEP_PARAMETER_MASK_PULSE_WIDTH){
						pulseWidth += getStepParameter(STEP_PARAMETER_PULSE_WIDTH,activePart,lane,activeStep);
					}


					if(!(sceneData[activeScene].lane[activePart][lane].laneClock%repeatTicks)){
						gateBuffer[j] |= (1<<i);
					}
					else if((pulseWidth*repeatTicks/16)-(sceneData[activeScene].lane[activePart][lane].laneClock%(repeatTicks))>=0){
						gateBuffer[j] |= (1<<i);
					}
					else{
						gateBuffer[j] &= ~(1<<i);
					}
				}
				else{
					gateBuffer[j] &= ~(1<<i);
				}

			}
		}
	}
	else if(transportState == TRANSPORT_STATE_STOP){
		for(int j = 0; j < 2; j++){
			gateBuffer[j] = 0;
		}
	}

	for(int j = 0; j < 2; j++){
		if((gateBuffer[j]!=previousgateBuffer[j])){
			return true;
		}
	}
	return false;
}

void Scene::setStepIndex(uint8_t part, uint8_t lane, uint8_t step, int8_t value){
	if((sceneData[activeScene].lane[part][lane].indexOffset + value) < 0){
		sceneData[activeScene].lane[part][lane].index[step] = 0;
	}
	else if((sceneData[activeScene].lane[part][lane].indexOffset + value) > maxScaleIndex){
		sceneData[activeScene].lane[part][lane].index[step] = maxScaleIndex;
	}
	else{
		sceneData[activeScene].lane[part][lane].index[step] = value;
	}
}
inline uint8_t Scene::getStepIndex(uint8_t part, uint8_t lane, uint8_t step){
	if((sceneData[activeScene].lane[part][lane].index[step] + sceneData[activeScene].lane[part][lane].indexOffset) < 0){
		return 0;
	}
	else if((sceneData[activeScene].lane[part][lane].index[step] + sceneData[activeScene].lane[part][lane].indexOffset) > maxScaleIndex){
		return maxScaleIndex;
	}
	else{
		return (sceneData[activeScene].lane[part][lane].index[step]);
	}
}
void Scene::setStepGlide(uint8_t part, uint8_t lane, uint8_t step, int8_t value){
	if(value >= glideBufferSize){
		//sceneData[activeScene].lane[part][lane].glide[step] = glideBufferSize-1;
	}
	else if(value < 0){
		//sceneData[activeScene].lane[part][lane].glide[step] = 0;
	}
	else{
		//sceneData[activeScene].lane[part][lane].glide[step] = value;
	}
}

bool Scene::updateCvBuffer(void){
	float incrementor = 1/glideBufferSize;

	static uint8_t indexBuffer[4];
	uint8_t previousIndexBuffer[4];
	for(int i = 0; i < 4; i++){
		previousIndexBuffer[i] = indexBuffer[i];
		indexBuffer[i] = getStepIndex(activePart,i, sceneData[activeScene].lane[activePart][i].activeStep );
		indexBuffer[i] += sceneData[activeScene].lane[activePart][i].indexOffset;
		cvBuffer[i][0] = scales[0][indexBuffer[i]];
	}
	for(int i = 0; i < 4; i++){
		if(indexBuffer[i] != previousIndexBuffer[i]){
			return true;
		}
	}
	return false;


	/* Glide function prototype:
	for(int i = 0; i < 16; i++){
		uint8_t activeStep = sceneData[activeScene].lane[activePart][i].activeStep;
		uint8_t startIndex = getStepIndex(activePart,i,activeStep);



		if(startIndex != getStepIndex(activePart,i,activeStep+1)){
			uint16_t startValue = scales[0][startIndex];
			uint16_t endValue = scales[0][getStepIndex(activePart,i,activeStep+1)];
			int32_t difference = (endValue - startValue);
			std::cout << "step: " << +activeStep;
			std::cout << " start: " << +startValue << " end:" << +endValue << " difference: " << +difference << std::endl;

			if(getStepParameter(STEP_PARAMETER_LENGTH,activePart,i,activeStep)){
				uint8_t stepSection = (sceneData[activeScene].lane[activePart][i].laneClock/(stepTicks));
				uint8_t length = getStepParameter(STEP_PARAMETER_LENGTH,activePart,i,activeStep) + 1;

				std::cout << " section: " << +stepSection << " length:" << +length << " lane clock: " << +sceneData[activeScene].lane[activePart][i].laneClock << std::endl;
				// Recalculate beginning and end of ramp
				startValue = startValue + stepSection*(difference/length);
				difference = difference/length;

				for(uint8_t j = 0; j < glideBufferSize; j++){
					cvBuffer[i][j] = startValue + (difference*j/glideBufferSize);
				}
			}
			else{
				for(uint8_t j = 0; j < glideBufferSize; j++){
					cvBuffer[i][j] = startValue + (difference*j/glideBufferSize);
				}
			}

		}
		else{
			for(uint8_t j = 0; j < glideBufferSize; j++){
				cvBuffer[i][j] = scales[0][startIndex];
			}
		}
	}
	 */
}

void Scene::updateLaneClocks(void){
	switch(transportState){
	case TRANSPORT_STATE_PAUSE:
		sceneData[activeScene].mainClock++;
		break;
	case TRANSPORT_STATE_STOP:
		sceneData[activeScene].mainClock++;
		for(int i = 0; i < 16; i++){
			sceneData[activeScene].lane[activePart][i].laneClock=0;
			sceneData[activeScene].lane[activePart][i].activeStep = 0;
		}
		break;
	case TRANSPORT_STATE_PLAY:
		sceneData[activeScene].mainClock++;
		for(int i = 0; i < 16; i++){
			int activeStep = sceneData[activeScene].lane[activePart][i].activeStep;
			int length = 1;
			length += getStepParameter(STEP_PARAMETER_LENGTH,activePart,i,activeStep);
			length += getLaneParameter(LANE_PARAMETER_DIVISION, activePart, i);
			if(++sceneData[activeScene].lane[activePart][i].laneClock >= (stepTicks*length)){
				// End of step
				sceneData[activeScene].lane[activePart][i].laneClock=0;

				if(++sceneData[activeScene].lane[activePart][i].activeStep > sceneData[activeScene].lane[activePart][i].endStep){
					// End of lane
					sceneData[activeScene].lane[activePart][i].activeStep = 0;
					if((focusedPart!=activePart) && playMode == PLAY_MODE_FOLLOW_FOCUS){
						activePart = focusedPart;
					}

				}
			}
		}
		break;
	case TRANSPORT_STATE_RESET:
		sceneData[activeScene].mainClock=0;
		for(int i = 0; i < 16; i++){
			sceneData[activeScene].lane[activePart][i].laneClock = 0;
			sceneData[activeScene].lane[activePart][i].activeStep = 0;
		}
		transportState = TRANSPORT_STATE_PLAY;
		break;
	}

}
uint8_t Scene::getLaneActiveStep(void){
	return sceneData[activeScene].lane[activePart][focusedLane].activeStep;
}
uint8_t Scene::getLaneActiveStep(uint8_t lane){
	return sceneData[activeScene].lane[activePart][lane].activeStep;
}


void Scene::selectionReset(void){
	selection.partMask = 0x0000;
	selection.laneMask = 0x0000;
	selection.stepMask = 0x0000;
	selection.notEmpty = false;
}

uint8_t Scene::selectionLength(SCOPE lengthScope, Selection sel){
	uint8_t selectionStart = 0, selectionEnd = 0;
	bool selectionStartFound = false, selectionEndFound = false;
	uint8_t maskEnd = 0;
	uint32_t selectionMask;
	switch(lengthScope){
	case SCOPE_SCENE:
		return 0;
	case SCOPE_PART:
		selectionMask = sel.partMask;
		maskEnd = 15;
		break;
	case SCOPE_LANE:
		selectionMask = sel.laneMask;
		maskEnd = 15;
		break;
	case SCOPE_STEP:
		selectionMask = sel.stepMask;
		maskEnd = 31;
		break;
	}
	for(uint8_t i = 0; i <= maskEnd; i++){
		if(!(selectionStartFound) && (selectionMask & (1 << i))){
			selectionStart = i;
			selectionStartFound = true;
		}
		if(!(selectionEndFound) && (selectionMask & (1 << (maskEnd-i)))){
			selectionEnd = (maskEnd-i);
			selectionEndFound = true;
		}
		if(selectionStartFound && selectionEndFound){
			return (selectionEnd - selectionStart) + 1;
		}
	}
	return 0;
}
uint8_t Scene::selectionStart(SCOPE startScope, Selection sel){
	uint8_t maskEnd = 0;
	uint32_t selectionMask;
	switch(startScope){
	case SCOPE_SCENE:
		return 0;
	case SCOPE_PART:
		selectionMask = sel.partMask;
		maskEnd = 15;
		break;
	case SCOPE_LANE:
		selectionMask = sel.laneMask;
		maskEnd = 15;
		break;
	case SCOPE_STEP:
		selectionMask = sel.stepMask;
		maskEnd = 31;
		break;
	}
	for(uint8_t i = 0; i <= maskEnd; i++){
		if(selectionMask & (1 << i)){
			return i;
		}
	}
}
uint8_t Scene::selectionEnd(SCOPE endScope, Selection sel){
	uint8_t maskEnd = 0;
	uint32_t selectionMask;
	switch(endScope){
	case SCOPE_SCENE:
		return 0;
	case SCOPE_PART:
		selectionMask = sel.partMask;
		maskEnd = 15;
		break;
	case SCOPE_LANE:
		selectionMask = sel.laneMask;
		maskEnd = 15;
		break;
	case SCOPE_STEP:
		selectionMask = sel.stepMask;
		maskEnd = 31;
		break;
	}
	for(uint8_t i = maskEnd; i == 0; i--){
		if(selectionMask & (1 << i)){
			return i;
		}
	}
}
void Scene::selectAllInFocus(SCOPE scope){
	selection.scope = scope;
	selection.scene = focusedScene;
	selection.notEmpty = true;
	switch(scope){
	case SCOPE_SCENE:
		selection.scene = focusedScene;
		selection.partMask = 0x0000FFFF;
		selection.laneMask = 0x0000FFFF;
		selection.stepMask = 0xFFFFFFFF;
		break;
	case SCOPE_PART:
		selection.scene = focusedScene;
		selection.partMask = (1 << focusedPart);
		selection.laneMask = 0x0000FFFF;
		selection.stepMask = 0xFFFFFFFF;
		break;
	case SCOPE_LANE:
		selection.scene = focusedScene;
		selection.partMask = (1 << focusedPart);
		selection.laneMask = (1 << focusedLane);
		selection.stepMask = 0xFFFFFFFF;
		break;
	case SCOPE_STEP:
		selection.scene = focusedScene;
		selection.partMask = (1 << focusedPart);
		selection.laneMask = (1 << focusedLane);
		selection.stepMask = (1 << focusedStep);
		break;
	}
}
bool Scene::copy(void){
	if(selection.notEmpty){
		copySelection = selection;
		return true;
	}
	else{
		return false;
	}
}

bool Scene::setPasteDestination(SCOPE destinationScope, uint8_t destination){
	switch(destinationScope){
	case SCOPE_SCENE:
		destinationScene = destination;
		break;
	case SCOPE_PART:
		destinationScene = focusedScene;
		destinationPart = destination;
		break;
	case SCOPE_LANE:
		destinationScene = focusedScene;
		destinationPart = focusedPart;
		destinationLane = destination;
		break;
	case SCOPE_STEP:
		destinationScene = focusedScene;
		destinationPart = focusedPart;
		destinationLane = focusedLane;
		destinationStep = destination;
		break;
	}
}
bool Scene::paste(void){
	uint8_t source, destination;
	switch(copySelection.scope){
	case SCOPE_SCENE:
		sceneData[destinationScene] = sceneData[copySelection.scene];
		break;
	case SCOPE_PART:
		for(uint8_t part = 0; part < selectionLength(SCOPE_PART, copySelection); part++){
			source = part + selectionStart(SCOPE_PART, copySelection);
			destination = part + destinationPart;

			if(copySelection.partMask & (1 << source)){
				for(uint8_t lane = 0; lane < 16; lane++){
					sceneData[destinationScene].lane[destination][lane] = sceneData[copySelection.scene].lane[source][lane];
				}
			}
		}
		break;
	case SCOPE_LANE:
		break;
	case SCOPE_STEP:
		break;
	}
}
