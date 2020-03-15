/*
 * sequencer.hpp
 *
 *  Created on: Feb 4, 2019
 *      Author: delta
 */



#include <vector>
#include <cstring>
#include <stdlib.h>
#include <switches.hpp>
#include <switches.hpp>
#include "stm32f4xx_hal.h"

#ifndef SEQUENCER_HPP_
#define SEQUENCER_HPP_

extern UART_HandleTypeDef huart2;

const uint8_t PLAY = (1 << 0);
const uint8_t PAUSE = (1 << 1);
const uint8_t COPY = (1 << 2);
const uint8_t PASTE = (1 << 3);
const uint8_t SELECT_LANE = (1 << 4);
const uint8_t SELECT_BLOCK = (1 << 5);
const uint8_t SELECT_NOTE = (1 << 6);
const uint8_t SELECT_SONG = (1 << 7);

enum TRANSPORT_STATE{
	TRANSPORT_STATE_STOP,
	TRANSPORT_STATE_PAUSE,
	TRANSPORT_STATE_PLAY,
	TRANSPORT_STATE_HOLD,
	TRANSPORT_STATE_RESUME,
	TRANSPORT_STATE_RESET
};
enum TRANSPORT_MODE{
	TRANSPORT_MODE_CONTINUE,
	TRANSPORT_MODE_REPEAT,
	TRANSPORT_MODE_MOVE,
	TRANSPORT_MODE_MOVE_ON_END,
};
enum STEPSWITCH_MODE{
	STEPSWITCH_SELECT_NOTE,
	STEPSWITCH_SELECT_LANE,
	STEPSWITCH_SELECT_BLOCK,
	STEPSWITCH_SELECT_SONG,
	STEPSWITCH_EDIT_BLOCK,
	STEPSWITCH_EDIT_SONG,
	STEPSWITCH_SET_END,
	STEPSWITCH_SET_PLAYING,
	STEPSWITCH_SET_MUTE
};

struct Song{
	char title[16];

	uint8_t firstStep, activeBlock;
	uint8_t activeStep[16], activeSubStep, lastStep[16][16], laneDivision[16][16];

	// 8 lanes, 16 blocks (c.q. patterns)

	// 16 lanes, 16 blocks, 16 notes with 24 subdivisions + 8 leftover bits?
	uint32_t subSteps[16][16][32];
	//uint8_t notes[16][16][32];
};

class Sequencer{
public:
	uint8_t selectedSong, selectedLane, selectedBlock, selectedStep;
	uint8_t playingSong, playingBlock;

	uint16_t playingLanes;
	uint16_t mutedLanes;

	TRANSPORT_STATE transportState = TRANSPORT_STATE_PLAY;
	TRANSPORT_MODE transportMode;

	STEPSWITCH_MODE stepSwitchMode = STEPSWITCH_EDIT_BLOCK;


	Song song[2];

	uint8_t* getPattern(void);

	Sequencer(void);
	~Sequencer(void);

	// Implement navigation/transport/menu/multifunction options
	void readControlSwitches(Switches& controlSwitches, Switches& stepSwitches);
	void writeTrigPattern(void);
	// Advance to next step depending on playState
	void stepTick(void);
	void subStepAdvance(void);
	// Decide what to do with this
	void masterTick(void);

};

#endif /* SEQUENCER_HPP_ */
