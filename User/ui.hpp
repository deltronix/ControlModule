/*
 * ui.hpp
 *
 *  Created on: Dec 10, 2019
 *      Author: delta
 */

#ifndef UI_HPP_
#define UI_HPP_

#include "switches.hpp"
#include "scene.hpp"
#include "memory.hpp"
#include "display.hpp"
#include "clock.hpp"



enum STEP_SWITCH_MODE{
	STEP_SWITCH_MODE_EDIT,
	STEP_SWITCH_MODE_SELECT,
	STEP_SWITCH_MODE_COPY,
	STEP_SWITCH_MODE_PASTE,
	STEP_SWITCH_MODE_DELETE,
	STEP_SWITCH_MODE_SET_END,
	STEP_SWITCH_MODE_MUTE
};

const char* const SCOPE_STRING[] = {
	"SCNE",
	"PART",
	"LANE",
	"STEP",
};

enum FOCUS{
	FOCUS_SCENE,
	FOCUS_PART,
	FOCUS_LANE,
	FOCUS_STEP,
};
enum UI_STATE{
	UI_STATE_SELECT,
	UI_STATE_EDIT,
	UI_STATE_NOTIFY_SAVING,
	UI_STATE_SAVING,
	UI_STATE_NOTIFY_LOADING,
	UI_STATE_LOADING,
	UI_STATE_SETTINGS,
	UI_STATE_OVERVIEW,
	UI_STATE_COPY,
	UI_STATE_PASTE,
};




class UserInterface{
public:
	UserInterface(void);
	~UserInterface(void);

	STEP_SWITCH_MODE stepSwitchMode = STEP_SWITCH_MODE_EDIT;
	UI_STATE previousUiState;
	UI_STATE uiState = UI_STATE_OVERVIEW;
	SCOPE scope = SCOPE_STEP;
	SCOPE previousScope;

	bool shiftPressed;
	bool saveMode;

	void changeState(UI_STATE state);
	void changeScope(SCOPE scope);

	void readSwitches(Switches& controlSwitches,Switches& stepSwitches, Scene& scene);
	void readEncoders(Encoder& encoderA, Encoder& encoderB, Scene& scene);
	void setStepSwitchLeds(Switches& stepSwitches, Scene& scene);
	void updateDisplay(Display& display, Scene& scene);
	void updateDisplayAndMemoryDma(Display& display, Memory& memory, Scene& scene, Clock& clock, Switches& controlSwitches);
};



#endif /* UI_HPP_ */
