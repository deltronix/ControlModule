/*
 * main.hpp
 *
 *  Created on: Oct 15, 2019
 *      Author: delta
 */

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include <stdint.h>

const uint8_t nSteps = 32;
const uint8_t nLanes = 16;
const uint8_t nParts = 16;
const uint8_t maxScaleIndex = 127;
const uint8_t glideBufferSize = 16;

enum STEP_PARAMETER{
	STEP_PARAMETER_NOTE_ON = 0,
	STEP_PARAMETER_REPEAT = 1,
	STEP_PARAMETER_PULSE_WIDTH = 4,
	STEP_PARAMETER_LENGTH = 8,
	STEP_PARAMETER_PROBABILITY = 12,
	STEP_PARAMETER_INDEX = 15,
	STEP_PARAMETER_GLIDE = 23,
	STEP_PARAMETER_MOD = 24
};
enum LANE_NOTE_STATE{
	LANE_NOTE_UNCHANGED,
	LANE_NOTE_ON,
	LANE_NOTE_OFF,
};
enum STEP_PARAMETER_MAX{
	STEP_PARAMETER_MAX_NOTE_ON 		=	(0x1),
	STEP_PARAMETER_MAX_REPEAT 		=	(0x7),
	STEP_PARAMETER_MAX_PULSE_WIDTH 	=	(0xF),
	STEP_PARAMETER_MAX_LENGTH 		=	(0xF),
	STEP_PARAMETER_MAX_PROBABILITY 	=	(0xF)
};
enum STEP_PARAMETER_MASK{
	STEP_PARAMETER_MASK_NOTE_ON 	=	(STEP_PARAMETER_MAX_NOTE_ON	<< STEP_PARAMETER_NOTE_ON),
	STEP_PARAMETER_MASK_REPEAT 		=	(STEP_PARAMETER_MAX_REPEAT << STEP_PARAMETER_REPEAT),
	STEP_PARAMETER_MASK_PULSE_WIDTH	= 	(STEP_PARAMETER_MAX_PULSE_WIDTH << STEP_PARAMETER_PULSE_WIDTH),
	STEP_PARAMETER_MASK_LENGTH 		=	(STEP_PARAMETER_MAX_LENGTH << STEP_PARAMETER_LENGTH),
	STEP_PARAMETER_MASK_PROBABILITY =	(STEP_PARAMETER_MAX_PROBABILITY << STEP_PARAMETER_PROBABILITY)
};
enum LANE_PARAMETER{
	LANE_PARAMETER_END_STEP,
	LANE_PARAMETER_DIVISION,
	LANE_PARAMETER_INDEX_OFFSET,
	LANE_PARAMETER_SCALE,
	LANE_PARAMETER_CV_MODE,
	LANE_PARAMETER_MIDI_CHAN,
	LANE_PARAMETER_MUTE,
	SIZE_OF_LANE_PARAMETERS,
};
enum MIDI_CHAN{
	MIDI_CHAN_1, MIDI_CHAN_2, MIDI_CHAN_3, MIDI_CHAN_4, MIDI_CHAN_5, MIDI_CHAN_6, MIDI_CHAN_7, MIDI_CHAN_8,
	MIDI_CHAN_9, MIDI_CHAN_10, MIDI_CHAN_11, MIDI_CHAN_12, MIDI_CHAN_13, MIDI_CHAN_14, MIDI_CHAN_15, MIDI_CHAN_16,
	MIDI_CHAN_OFF,

};
enum TRANSPORT_STATE{
	TRANSPORT_STATE_PLAY,
	TRANSPORT_STATE_STOP,
	TRANSPORT_STATE_PAUSE,
	TRANSPORT_STATE_RESET,
};
enum SCOPE{
	SCOPE_SCENE,
	SCOPE_PART,
	SCOPE_LANE,
	SCOPE_STEP,
};
enum PLAY_MODE{
	PLAY_MODE_FOLLOW_FOCUS,
	PLAY_MODE_LOOP_ACTIVE,
	PLAY_MODE_FOLLOW_COMPOSITION,
	PLAY_MODE_CUE_SCENE,
};

struct Lane{
	// Step data
	uint16_t step[nSteps] = {0};
	int8_t index[nSteps] = {0};
	int8_t glide[nSteps] = {0};
	int8_t mod[nSteps] = {0};

	// Parameters
	uint8_t endStep = nSteps-1;
	uint8_t indexOffset = 60;
	uint8_t division = 0;
	uint8_t midiChan = MIDI_CHAN_OFF;
	uint8_t scale = 0;

	// Internal counter
	uint8_t activeStep = 0;
	uint16_t laneClock = 0;
};

struct SceneMetaData{
	uint8_t identifier = 1;
	uint8_t tempo = 120;
	uint32_t memoryAddress;
	char sceneName[16] = "Test Scene";
};

struct SceneData{
	// This struct is to be stored in internal FLASH (as well?)
	SceneMetaData metaData;
	Lane lane[16][16];
	uint16_t laneMute[16] = {0};

	// | B7 | B5: Reset Lane Clocks | B4: Loop | B3-B0 Part |
	uint8_t composition[16];
	uint8_t repeats[16];
	uint8_t repeatCounter;
	uint8_t position;

	uint32_t mainClock;
};
struct Selection{
	SCOPE scope;
	bool scene;
	bool notEmpty = false;
	uint16_t partMask;
	uint16_t laneMask;
	uint32_t stepMask;
};

class Scene{
public:
	uint8_t activeScene, activePart, focusedPart, focusedScene, focusedLane, focusedStep;



	STEP_PARAMETER focusedStepParameter;
	LANE_PARAMETER focusedLaneParameter;

	TRANSPORT_STATE transportState = TRANSPORT_STATE_STOP;
	PLAY_MODE playMode = PLAY_MODE_FOLLOW_FOCUS;

	Scene(uint16_t beatClockTicks);
	~Scene();

	void focusPart(uint8_t part);
	void focusLane(uint8_t lane);
	void focusStep(uint8_t step);

	STEP_PARAMETER focusNextStepParameter(int16_t n);
	LANE_PARAMETER focusNextLaneParameter(int16_t n);

	const char*	stepParameterString();
	void incrementDecrementStepParameter(int16_t n);

	uint32_t getMainClock(void);
	uint8_t getStepParameterArray(STEP_PARAMETER param, uint8_t fromPart, uint8_t fromLane, uint8_t* valueArray);
	uint8_t getStepParameterArray(STEP_PARAMETER param, uint8_t* valueArray);
	uint8_t getStepNoteOnMask(uint8_t fromPart, uint8_t fromLane, uint8_t n, uint8_t* valueArray);
	uint8_t getStepNoteOnMask(uint8_t n, uint8_t* valueArray);
	void toggleStepNoteOn(uint8_t step);

	void setStepParameter(STEP_PARAMETER param, uint8_t step, int8_t value);
	void setStepParameter(STEP_PARAMETER param, uint8_t fromLane, uint8_t step, int8_t value);
	uint8_t getStepParameter(STEP_PARAMETER param, uint8_t step);
	uint8_t getStepParameter(STEP_PARAMETER param, uint8_t fromPart, uint8_t fromLane, uint8_t step);

	void setLaneParameter(LANE_PARAMETER param, uint8_t fromPart, uint8_t fromLane, uint8_t value);
	uint8_t getLaneParameter(LANE_PARAMETER param, uint8_t fromPart, uint8_t fromLane);
	void incrementDecrementLaneParameter(int n);

	void setStepIndex(uint8_t part, uint8_t lane, uint8_t step, int8_t value);
	void setStepGlide(uint8_t part, uint8_t lane, uint8_t step, int8_t value);
	inline uint8_t getStepIndex(uint8_t part, uint8_t lane, uint8_t step);

	bool updateCvBuffer(void);
	bool updateGateBuffer(uint16_t clock);

	void updateLaneClocks(void);
	uint8_t getLaneActiveStep(void);
	uint8_t getLaneActiveStep(uint8_t lane);

	LANE_NOTE_STATE laneNoteState(uint8_t lane);

	uint8_t gateBuffer[2], previousgateBuffer[2];
	uint16_t cvBuffer[nSteps][glideBufferSize];

	SceneData sceneData[2];
	SceneData& activeScenePointer(void);
	SceneData& idleScenePointer(void);
	uint8_t switchActiveScene(void);


	/*			--- SELECTION --- 			*/
	Selection selection, copySelection;
	void selectByMask(SCOPE scope, uint8_t* selectionMask);
	void selectAllInFocus(SCOPE scope);
	void selectionReset(void);
	uint8_t selectionLength(SCOPE lengthScope, Selection sel);
	uint8_t selectionStart(SCOPE startScope, Selection sel);
	uint8_t selectionEnd(SCOPE endScope, Selection sel);


	bool copy(void);
	uint8_t destinationScene, destinationPart, destinationLane, destinationStep;

	bool setPasteDestination(SCOPE destinationScope, uint8_t destination);
	bool paste(void);

private:


	uint16_t beatTicks;
	uint16_t stepTicks;


};

const char* const LaneParameters[] =
{
	"END",
	"DIV",
	"OFFSET",
	"SCALE",
	"CVMODE",
	"MIDICH",
	"MUTE",
};

#endif /* SCENE_HPP_ */
