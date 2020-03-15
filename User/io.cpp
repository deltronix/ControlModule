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
							midiNoteOn(channel, scene.getStepParameter(STEP_PARAMETER_INDEX,scene.activePart,lane,scene.getLaneActiveStep(lane)), 60);
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

