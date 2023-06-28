#include "io.hpp"

const uint16_t IO::scales[2][128] = {{
		// Twelve tone equal temperament binary codes, indexed by MIDI note
		0,     390,   803,   1240,  1703,  2194,  2715,  3266,  3850,  4468,  5124,  5818,  // C-1
		6553,  6943,  7356,  7793,	8257,  8748,  9268,  9819,	10403, 11022, 11677, 12371, // C0 [12]
		13107, 13497, 13910, 14347, 14810, 15301, 15822, 16373, 16957, 17575, 18231, 18925, // C1 [24]
		19660, 20050, 20463, 20900, 21364, 21855, 22375, 22926, 23510, 24129, 24784, 25478, // C2 [36]
		26214, 26604, 27017, 27454, 27917, 28408, 28929, 29480, 30064, 30682, 31338, 32032, // C3 [48]
		32767, 33157, 33570, 34007, 34471, 34962, 35482, 36033, 36617, 37236, 37891, 38585, // C4 [60] (0V)
		39321, 39711, 40124, 40561, 41024, 41515, 42036, 42587,	43171, 43789, 44445, 45139, // C5 [72]
		45874, 46264, 46677, 47114, 47578, 48069, 48589, 49140, 49724, 50343, 50998, 51692, // C6 [84]
		52428, 52818, 53231, 53668,	54131, 54622, 55143, 55694, 56278, 56896, 57552, 58246, // C7 [96]
		58981, 59371, 59784, 60221, 60685, 61176, 61696, 62247, 62831, 63450, 64105, 64799, // C8 [108]
		65535},{
}

};



IO::IO(SPI_HandleTypeDef* spi, UART_HandleTypeDef* uart){
	hspi = spi;
	huart = uart;
}

IO::~IO(void){
	;
}
IO_STATE IO::dmaStateMachine(void){
	if(dmaTransferInProgress){
		switch(ioState){
		case IO_READY:
			break;
		case IO_ANALOG_WRITE:
			HAL_GPIO_WritePin(SPI3_SYNC_GPIO_Port,SPI3_SYNC_Pin,GPIO_PIN_SET);
			break;
		case IO_DIGITAL_WRITE:
			HAL_GPIO_WritePin(SPI3_RCLK_GPIO_Port,SPI3_RCLK_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SPI3_RCLK_GPIO_Port,SPI3_RCLK_Pin,GPIO_PIN_SET);
			ioState = IO_DONE;
			break;
		}

	}
	switch(ioState){
	case IO_READY:
		dmaChannel = 0;
		writeDacDma();
		ioState = IO_ANALOG_WRITE;
		break;
	case IO_ANALOG_WRITE:
		if(++dmaChannel < 4){
			writeDacDma();
		}
		else{
			ioState = IO_DIGITAL_WRITE;
			HAL_GPIO_WritePin(SPI3_SYNC_GPIO_Port,SPI3_SYNC_Pin,GPIO_PIN_SET);
			writeDigitalDma();
		}
		break;
	case IO_DIGITAL_WRITE:
		break;
	case IO_DONE:
		dmaTransferInProgress = false;
		ioState = IO_READY;
		break;
	}
	return ioState;
}

void IO::setBufferParameters(Scene& scene){
	static bool setBufferParametersDone;

	switch(scene.transportState){
	case TRANSPORT_STATE_RESET:
		dmaSample = 0;
		setBufferParametersDone = false;
		break;
	case TRANSPORT_STATE_STOP:
		if(!setBufferParametersDone){
			activeAnalogBuffer = 0;
			for(uint8_t lane = 0; lane < 4; lane++){
				analogBuffer[lane][activeAnalogBuffer].position = 0;
				waitingForBufferUpdate[activeAnalogBuffer] = true;
			}
			setBufferParametersDone = true;
		}
		break;
	case TRANSPORT_STATE_PLAY:
		dmaSample++;
		if(dmaSample < 96){

		}
		else{
			dmaSample = 0;
			for(uint8_t lane = 0; lane < 4; lane++){
				analogBuffer[lane][activeAnalogBuffer].position+= 96;
			}
			activeAnalogBuffer ^= 1;
			setBufferParametersDone = false;
		}
		if(!setBufferParametersDone){
			for(uint8_t lane = 0; lane < 4; lane++){
				waitingForBufferUpdate[activeAnalogBuffer^1] = true;
				setBufferParametersDone = true;
			}
		}
		break;
	}

}

void IO::startDmaTransfer(Scene& scene){
	if(!dmaTransferInProgress){
		dmaStateMachine();
	}


}
void IO::writeDigitalDma(void){
	dmaTransferInProgress = true;
	HAL_GPIO_WritePin(SPI3_SYNC_GPIO_Port,SPI3_SYNC_Pin,GPIO_PIN_SET);
	HAL_SPI_Transmit_IT(hspi, digitalBuffer,2);

}

void IO::writeDacDma(void){
	dmaBuffer[0] = 0;
	dmaBuffer[1] = dmaChannel;
	dmaBuffer[2] = (analogBuffer[dmaChannel][activeAnalogBuffer].samples[dmaSample] & 0xFF00) >> 8;
	dmaBuffer[3] = (analogBuffer[dmaChannel][activeAnalogBuffer].samples[dmaSample] & 0x00FF);
	HAL_GPIO_WritePin(SPI3_SYNC_GPIO_Port,SPI3_SYNC_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(hspi, dmaBuffer, 4);
	dmaTransferInProgress = true;
}

void IO::setDacRegister(uint8_t reg, uint8_t chan, uint16_t val){
	uint8_t buffer[4] = {0};
	HAL_GPIO_WritePin(SPI3_SYNC_GPIO_Port,SPI3_SYNC_Pin,GPIO_PIN_SET);
	buffer[0] = 0;
	buffer[1] |= (reg & REGISTER_BITS) | (chan & CHANNEL_BITS);
	buffer[2] = (val & 0xFF00) >> 8;
	buffer[3] = (val & 0x00FF);

	HAL_GPIO_WritePin(SPI3_SYNC_GPIO_Port,SPI3_SYNC_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi,buffer,4,1);
	HAL_GPIO_WritePin(SPI3_SYNC_GPIO_Port,SPI3_SYNC_Pin,GPIO_PIN_SET);
}
void IO::setDigitalOuts(uint8_t* val, uint8_t n){
	HAL_GPIO_WritePin(SPI3_SYNC_GPIO_Port,SPI3_SYNC_Pin,GPIO_PIN_SET);
	HAL_SPI_Transmit(hspi,val,n,1);
	HAL_GPIO_WritePin(SPI3_RCLK_GPIO_Port,SPI3_RCLK_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI3_RCLK_GPIO_Port,SPI3_RCLK_Pin,GPIO_PIN_SET);

}
void IO::writeDacs(void){

	for(int i = 0; i < 4; i++){
		if(dacOutBuffer[i]!=dacOutState[i]){
			setDacRegister(DAC_REGISTER,i,dacOutBuffer[i]);
			dacOutState[i] = dacOutBuffer[i];
		}
	}
	// ---------------------------- SWITCH FUNCTIONS  ---------------
}
void IO::initDacs(uint8_t range){
	uint8_t buffer[4] = {0,0,0,0};
	HAL_SPI_Transmit(hspi,buffer,4,1);

	setDacRegister(RANGE_REGISTER,DAC_CHANNEL_A,RANGE_PLUS_MINUS_5);
	setDacRegister(RANGE_REGISTER,DAC_CHANNEL_B,RANGE_PLUS_MINUS_5);
	setDacRegister(RANGE_REGISTER,DAC_CHANNEL_C,RANGE_PLUS_MINUS_5);
	setDacRegister(RANGE_REGISTER,DAC_CHANNEL_D,RANGE_PLUS_MINUS_5);

	setDacRegister(POWER_REGISTER,0,POWER_UP_ALL_DACS);
	HAL_Delay(1);
}


// Calculate DAC samples for the next step.
bool IO::updateAnalogBuffer(Scene& scene){
	for(uint8_t buffer = 0; buffer < 2; buffer++){
		if(waitingForBufferUpdate[buffer]){
			for(uint8_t lane = 0; lane < 4; lane++){
				uint8_t step;
				uint32_t glideLength = 0;


				if(buffer == activeAnalogBuffer){
					step = scene.getLaneActiveStep(lane);
					analogBuffer[lane][buffer].position = 0;
					analogBuffer[lane][buffer].startValue = scales[scene.getLaneParameter(LANE_PARAMETER_SCALE, scene.activePart, lane)]
																   [scene.getStepParameter(STEP_PARAMETER_INDEX, scene.activePart, lane, step)];
					uint8_t i = 0;
					for(bool endFound = false; endFound != true;){
						uint8_t seekEndStep = (step + i) % scene.getLaneParameter(LANE_PARAMETER_END_STEP, scene.activePart, lane);

						if(!(scene.getStepParameter(STEP_PARAMETER_NOTE_ON, scene.activePart, lane, seekEndStep))){
							glideLength += (1 + scene.getStepParameter(STEP_PARAMETER_LENGTH, scene.activePart, lane, seekEndStep));
						}
						else if(i > nSteps){
							analogBuffer[lane][buffer].endValue = scales[scene.getLaneParameter(LANE_PARAMETER_SCALE, scene.activePart, lane)]
																		 [scene.getStepParameter(STEP_PARAMETER_INDEX, scene.activePart, lane, seekEndStep)];
							// Set length according to lane subdivision and the numbers of ticks per step
							analogBuffer[lane][buffer].length = glideLength * 96 * (1 + scene.getLaneParameter(LANE_PARAMETER_DIVISION, scene.activePart, lane));
							analogBuffer[lane][buffer].incrementor = (analogBuffer[lane][buffer].endValue - analogBuffer[lane][buffer].startValue)
																															/ analogBuffer[lane][buffer].length;
							endFound = true;
						}
						else{
							analogBuffer[lane][buffer].endValue = scales[scene.getLaneParameter(LANE_PARAMETER_SCALE, scene.activePart, lane)]
																		 [scene.getStepParameter(STEP_PARAMETER_INDEX, scene.activePart, lane, seekEndStep)];
							// Set length according to lane subdivision and the numbers of ticks per step
							analogBuffer[lane][buffer].length = glideLength * 96 * (1 + scene.getLaneParameter(LANE_PARAMETER_DIVISION, scene.activePart, lane));
							analogBuffer[lane][buffer].incrementor = (analogBuffer[lane][buffer].endValue - analogBuffer[lane][buffer].startValue)
																																	/ analogBuffer[lane][buffer].length;
							endFound = true;
						}
						i++;

					}
				}
				// Check if curve ends after this buffer
				else if(analogBuffer[lane][activeAnalogBuffer].position + nTicksPerStep >= analogBuffer[lane][activeAnalogBuffer].length){
					// Then make sure the inactive buffer calculates a new curve from the next step
					if(scene.getLaneActiveStep(lane) == scene.getLaneParameter(LANE_PARAMETER_END_STEP, scene.activePart, lane)){step = 0;}
					else{step = scene.getLaneActiveStep(lane) + 1;}

					// If the next step is not turned on, fill buffer with end value of last buffer
					if(!scene.getStepParameter(STEP_PARAMETER_NOTE_ON, scene.activePart, lane, step)){
						analogBuffer[lane][buffer].position = 0;
						analogBuffer[lane][buffer].startValue = analogBuffer[lane][buffer^1].endValue;
						analogBuffer[lane][buffer].endValue = analogBuffer[lane][buffer^1].endValue;
					}
					else{
						// Update start and end value, length and reset position when currently active buffer is done
						analogBuffer[lane][buffer].position = 0;
						analogBuffer[lane][buffer].startValue = scales[scene.getLaneParameter(LANE_PARAMETER_SCALE, scene.activePart, lane)]
																	   [scene.getStepParameter(STEP_PARAMETER_INDEX, scene.activePart, lane, step)];
						uint8_t i = 0;
						for(bool endFound = false; endFound != true;){
							uint8_t seekEndStep = (step + i) % scene.getLaneParameter(LANE_PARAMETER_END_STEP, scene.activePart, lane);

							if(!(scene.getStepParameter(STEP_PARAMETER_NOTE_ON, scene.activePart, lane, seekEndStep))){
								glideLength += (1 + scene.getStepParameter(STEP_PARAMETER_LENGTH, scene.activePart, lane, seekEndStep));
							}
							else if(i > nSteps){
								analogBuffer[lane][buffer].endValue = scales[scene.getLaneParameter(LANE_PARAMETER_SCALE, scene.activePart, lane)]
																			 [scene.getStepParameter(STEP_PARAMETER_INDEX, scene.activePart, lane, seekEndStep)];
								// Set length according to lane subdivision and the numbers of ticks per step
								analogBuffer[lane][buffer].length = glideLength * 96 * (1 + scene.getLaneParameter(LANE_PARAMETER_DIVISION, scene.activePart, lane));
								analogBuffer[lane][buffer].incrementor = (analogBuffer[lane][buffer].endValue - analogBuffer[lane][buffer].startValue)
																												/ analogBuffer[lane][buffer].length;
								endFound = true;
							}
							else{
								analogBuffer[lane][buffer].endValue = scales[scene.getLaneParameter(LANE_PARAMETER_SCALE, scene.activePart, lane)]
																			 [scene.getStepParameter(STEP_PARAMETER_INDEX, scene.activePart, lane, seekEndStep)];
								// Set length according to lane subdivision and the numbers of ticks per step
								analogBuffer[lane][buffer].length = glideLength * 96 * (1 + scene.getLaneParameter(LANE_PARAMETER_DIVISION, scene.activePart, lane));
								analogBuffer[lane][buffer].incrementor = (analogBuffer[lane][buffer].endValue - analogBuffer[lane][buffer].startValue)
																												/ analogBuffer[lane][buffer].length;
								endFound = true;
							}

						}
					}
				}
				else{
					// Continue the current curve
					analogBuffer[lane][buffer].position = analogBuffer[lane][buffer^1].position;
					analogBuffer[lane][buffer].startValue = analogBuffer[lane][buffer^1].startValue;
					analogBuffer[lane][buffer].startValue = analogBuffer[lane][buffer^1].endValue;
					analogBuffer[lane][buffer].length = analogBuffer[lane][buffer^1].length;

				}

				uint16_t bufferStart = analogBuffer[lane][buffer].startValue +
						(analogBuffer[lane][buffer].incrementor*analogBuffer[lane][buffer].position);

				for(uint16_t sample = 0; sample < nTicksPerStep; sample++){
					analogBuffer[lane][buffer].samples[sample] =
							bufferStart + (analogBuffer[lane][buffer].incrementor*sample);
				}

			}
			waitingForBufferUpdate[buffer] = false;
		}
	}

}



void IO::midiOut(Scene& scene){
	for(uint8_t i = 0; i < 2; i++){
		uint8_t changed = scene.gateBuffer[i] ^ scene.previousgateBuffer[i];

		if(changed){
			for(uint8_t j = 0; j < 8; j++){
				if(changed & (1 << j)){
					uint8_t lane = (i*8)+j;


					if(scene.gateBuffer[i] & (1 << j)){
						uint8_t channel = scene.getLaneParameter(LANE_PARAMETER_MIDI_CHAN, scene.activePart, lane);
						if(!(channel & (1 << 4))){
							midiNoteOn(channel,
									scene.getStepParameter(STEP_PARAMETER_INDEX,scene.activePart,lane,scene.getLaneActiveStep(lane)),
									scene.getStepParameter(STEP_PARAMETER_MOD,scene.activePart,lane,scene.getLaneActiveStep(lane)));
						}

					}
					else{
						// Note changed to off
					}
				}
			}
		}
	}
	if((midiTransmitMessages > 0) & !(midiTransferInProgress)){
		HAL_UART_Transmit_IT(huart, midiTransmitBuffer[midiTransmitIndex-midiTransmitMessages], 3);
		midiTransmitMessages--;
		midiTransferInProgress = true;
	}
}
void IO::midiTransmitDone(){
	if(midiTransmitMessages > 0){
		HAL_UART_Transmit_IT(huart, midiTransmitBuffer[midiTransmitIndex-midiTransmitMessages], 3);
		midiTransmitMessages--;
		midiTransferInProgress = true;

	}
	else{
		midiTransmitIndex = 0;
		midiTransferInProgress = false;
	}
}
void IO::midiNoteOn(uint8_t chan, uint8_t note, uint8_t vel){
	if(midiTransmitIndex < 63){
		midiTransmitBuffer[midiTransmitIndex][0] = 0x90 | (chan & 0x0F);
		midiTransmitBuffer[midiTransmitIndex][1] = note & 0x7F;
		midiTransmitBuffer[midiTransmitIndex][2] = vel & 0x7F;
		midiTransmitMessages++;
		midiTransmitIndex++;
	}
}
void IO::midiNoteOff(uint8_t chan, uint8_t note, uint8_t vel){
	if(midiTransmitIndex < 63){
		midiTransmitBuffer[midiTransmitIndex][0] = 0x80 | (chan & 0x0F);
		midiTransmitBuffer[midiTransmitIndex][1] = note & 0x7F;
		midiTransmitBuffer[midiTransmitIndex][2] = vel & 0x7F;
		midiTransmitMessages++;
		midiTransmitIndex++;
	}
}

