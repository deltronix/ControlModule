/*
 * memory.cpp
 *
 *  Created on: Dec 26, 2019
 *      Author: delta
 */

#include "memory.hpp"

Memory::Memory(SPI_HandleTypeDef* hspi, GPIO_TypeDef * chipSelectPort, uint16_t chipSelectPin){
	spi = hspi;
	CS_GPIO_Port = chipSelectPort;
	CS_GPIO_Pin = chipSelectPin;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
}
Memory::~Memory(void){

}

void Memory::sendInstruction(MEMORY_INSTRUCTION instruction, uint32_t address){
	uint8_t spi_out[5];
	spi_out[0] = instruction;
	switch(instruction){
	case WRITE_ENABLE:
	case VOLATILE_SR_WRITE_ENABLE:
	case WRITE_DISABLE:
	case GLOBAL_BLOCK_LOCK:
	case GLOBAL_BLOCK_UNLOCK:
	case ERASE_PROGRAM_SUSPEND:
	case ERASE_PROGRAM_RESUME:
	case POWER_DOWN:
	case ENABLE_RESET:
	case RESET_DEVICE:
	case CHIP_ERASE:
		// Single byte instructions
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(spi, spi_out, 1, 1);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
		break;
	case READ_STATUS_REGISTER_1:
	case READ_STATUS_REGISTER_2:
	case READ_STATUS_REGISTER_3:
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(spi, spi_out, 1, 10);
		break;
	case SECTOR_ERASE:
	case BLOCK_ERASE_32:
	case BLOCK_ERASE_64:
		spi_out[1] = (address >> 16) & 0xFF;
		spi_out[2] = (address >> 8) & 0xFF;
		spi_out[3] = address & 0xFF;
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(spi, spi_out, 4, 10);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
		break;
	case READ_DATA:
	case PAGE_PROGRAM:
		spi_out[1] = (address >> 16) & 0xFF;
		spi_out[2] = (address >> 8) & 0xFF;
		spi_out[3] = address & 0xFF;
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(spi, spi_out, 4, 10);
		break;
	case FAST_READ:
		spi_out[1] = (address >> 16) & 0xFF;
		spi_out[2] = (address >> 8) & 0xFF;
		spi_out[3] = address & 0xFF;
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(spi, spi_out, 5, 10);
		break;
	}
}
void Memory::writeByte(uint32_t address, uint8_t byte){
	sendInstruction(WRITE_ENABLE, 0);
	sendInstruction(PAGE_PROGRAM, address);
	HAL_SPI_Transmit(spi, &byte, 1, 1);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
}
uint8_t Memory::readByte(uint32_t address){
	uint8_t spi_in;
	sendInstruction(READ_DATA, address);
	HAL_SPI_Receive(spi, &spi_in, 1, 1);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
	return spi_in;
}
bool Memory::isBusy(){
	if(dmaTransferInProgress){return true;}
	else{
		uint8_t spi_in;
		sendInstruction(READ_STATUS_REGISTER_1,0);
		HAL_SPI_Receive(spi,&spi_in, 1, 10);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
		return (spi_in & SR1_BIT_BUSY);
	}

}
bool Memory::writeEnabled(){
	uint8_t spi_in;
	sendInstruction(READ_STATUS_REGISTER_1,0);
	HAL_SPI_Receive(spi,&spi_in, 1, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
	return (spi_in & SR1_BIT_WEL);
}
bool Memory::readyForWrite(void){
	uint8_t statusRegister1;

	sendInstruction(READ_STATUS_REGISTER_1,0);
	HAL_SPI_Receive(spi,&statusRegister1, 1, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);

	if(statusRegister1 & SR1_BIT_BUSY){busyBitState = true;}
	else{busyBitState = false;}

	if(statusRegister1 & SR1_BIT_WEL){writeEnableBitState = true;}
	else{writeEnableBitState = false;}

	return !busyBitState & writeEnableBitState;
}

void Memory::writeBlock64(uint32_t blockAddress, uint16_t nBytes, void* data){
	// Write multiple pages up to a block, the entire 64KB block will be erased in the progress
	uint16_t pageCounter = 0;
	uint16_t bytesRemainder = nBytes%256;
	uint16_t nPages = nBytes/256;
	uint32_t pageAddress = 0;

	//Only accept start of block
	blockAddress &= 0xFF0000;

	// Cast starting memory address to incrementable write pointer
	uint8_t* writePointer = static_cast<uint8_t*>(data);

	while(isBusy()){HAL_Delay(10);}
	sendInstruction(WRITE_ENABLE,0);
	while(!(writeEnabled())){};
	sendInstruction(BLOCK_ERASE_64,blockAddress);

	// Erasing a block takes about 2 seconds
	while(isBusy()){HAL_Delay(2000);}

	while(pageCounter < nPages){
		pageAddress = blockAddress | (pageCounter << 8);
		sendInstruction(WRITE_ENABLE, 0);
		while(!(writeEnabled())){};
		sendInstruction(PAGE_PROGRAM,pageAddress);
		HAL_SPI_Transmit(spi, writePointer, 256, 100);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);

		// Page write takes about 3ms
		while(isBusy()){HAL_Delay(5);}
		// Increment page counter and memory address pointer
		pageCounter++;
		writePointer+=256;
	}
	// Write remaining bytes
	pageAddress = blockAddress | (pageCounter << 8);
	sendInstruction(WRITE_ENABLE, 0);
	while(!(writeEnabled())){};
	sendInstruction(PAGE_PROGRAM,pageAddress);
	HAL_SPI_Transmit(spi, writePointer, bytesRemainder, 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);

	while(isBusy()){HAL_Delay(5);}
}
void Memory::readBlock(uint32_t blockAddress, uint16_t nBytes, void* data){
	uint16_t pageCounter = 0;
	uint16_t bytesRemainder = nBytes%256;
	uint16_t nPages = nBytes/256;
	uint32_t pageAddress = 0;

	//Only accept start of block
	blockAddress &= 0xFF0000;

	// Cast starting memory address to incrementable write pointer
	uint8_t* writePointer = static_cast<uint8_t*>(data);

	while(isBusy()){HAL_Delay(1);}
	while(pageCounter < nPages){
		pageAddress = blockAddress | (pageCounter << 8);
		sendInstruction(FAST_READ,pageAddress);
		HAL_SPI_Receive(spi, writePointer, 256, 100);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);

		// Page write takes about 3ms
		// Increment page counter and memory address pointer
		pageCounter++;
		writePointer+=256;
	}
	// Write remaining bytes
	pageAddress = blockAddress | (pageCounter << 8);
	sendInstruction(FAST_READ,pageAddress);
	HAL_SPI_Receive(spi, writePointer, bytesRemainder, 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
}

bool Memory::writeSceneDma(SceneData& sceneData){
	if(memoryState == MEMORY_READY){
		dmaPageCounter = 0;
		dmaBlockAddress = sceneData.metaData.memoryAddress & 0xFF0000;
		dmaPageAmount = sizeof(SceneData) / FLASH_PAGE_SIZE;
		dmaBytesRemainder = sizeof(SceneData) % FLASH_PAGE_SIZE;

		void* sceneDataPtr = &sceneData;
		dmaWritePointer = static_cast<uint8_t*>(sceneDataPtr);

		memoryState = MEMORY_WRITE_WAITING_FOR_ERASE;
		transferComplete = false;
		return true;
	}
	else{
		return false;
	}

}
bool Memory::readSceneDma(SceneData& sceneData){
	if(memoryState == MEMORY_READY){
		dmaPageCounter = 0;
		dmaBlockAddress = sceneData.metaData.memoryAddress & 0xFF0000;
		dmaPageAmount = sizeof(SceneData) / FLASH_PAGE_SIZE;
		dmaBytesRemainder = sizeof(SceneData) % FLASH_PAGE_SIZE;

		void* sceneDataPtr = &sceneData;
		dmaWritePointer = static_cast<uint8_t*>(sceneDataPtr);

		memoryState = MEMORY_WAITING_FOR_READ;
		transferComplete = false;
		return true;
	}
	else{
		return false;
	}
}
bool Memory::writePageDma(void){
	uint32_t dmaPageAddress;
	if(dmaPageCounter < dmaPageAmount){
		// Full page writes
		dmaPageAddress = dmaBlockAddress | (dmaPageCounter << 8);
		sendInstruction(PAGE_PROGRAM,dmaPageAddress);
		HAL_SPI_Transmit_DMA(spi, dmaWritePointer, FLASH_PAGE_SIZE);
		dmaPageCounter++;
		dmaWritePointer+=FLASH_PAGE_SIZE;
		dmaTransferInProgress = true;
		return false;
	}
	else if(dmaPageCounter == dmaPageAmount){
		// Write remaining bytes to the final page
		dmaPageAddress = dmaBlockAddress | (dmaPageCounter << 8);
		sendInstruction(PAGE_PROGRAM,dmaPageAddress);
		HAL_SPI_Transmit_DMA(spi, dmaWritePointer, dmaBytesRemainder);
		dmaPageCounter++;
		dmaTransferInProgress = true;
		return false;
	}
	else{
		// Transfer is done
		return true;
	}
}
bool Memory::readPageDma(void){
	uint32_t dmaPageAddress;
	if(dmaPageCounter < dmaPageAmount){
		// Full page writes
		dmaPageAddress = dmaBlockAddress | (dmaPageCounter << 8);
		sendInstruction(FAST_READ,dmaPageAddress);
		HAL_SPI_Receive_DMA(spi, dmaWritePointer, FLASH_PAGE_SIZE);
		dmaTransferInProgress = true;
		dmaPageCounter++;
		dmaWritePointer+=FLASH_PAGE_SIZE;
		return false;
	}
	else if(dmaPageCounter == dmaPageAmount){
		// Write remaining bytes to the final page
		dmaPageAddress = dmaBlockAddress | (dmaPageCounter << 8);
		sendInstruction(FAST_READ,dmaPageAddress);
		HAL_SPI_Receive_DMA(spi, dmaWritePointer, dmaBytesRemainder);
		dmaPageCounter++;
		dmaTransferInProgress = true;
		return false;
	}
	else{
		return true;
	}
}
void Memory::dmaTransferComplete(void){
	dmaTransferInProgress = false;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
}

void Memory::completeTransfer(){
	if(!isBusy()){
		transferComplete = true;
	}
}

MEMORY_STATE Memory::stateMachine(void){
	switch(memoryState){
	case MEMORY_READY:
		break;
	/* 		MEMORY WRITE STATES			*/
	case MEMORY_WRITE_WAITING_FOR_ERASE:
		if(readyForWrite()){
			sendInstruction(BLOCK_ERASE_64,dmaBlockAddress);
			memoryState = MEMORY_WRITE_ERASE_IN_PROGRESS;
		}
		else{
			sendInstruction(WRITE_ENABLE, 0);
		}
		break;
	case MEMORY_WRITE_ERASE_IN_PROGRESS:
		if(!isBusy()){
			memoryState = MEMORY_WAITING_FOR_WRITE;
		}
		break;
	case MEMORY_WAITING_FOR_WRITE:
		if(!isBusy()){
			sendInstruction(WRITE_ENABLE, 0);
			if(writeEnabled()){
			memoryState = MEMORY_WRITE_IN_PROGRESS;
			}
		}
		break;
	case MEMORY_WRITE_IN_PROGRESS:
		if(writePageDma()){
				if(!isBusy()){memoryState = MEMORY_WRITE_DONE;}
		}
		else{memoryState = MEMORY_WAITING_FOR_WRITE;}

		break;
	case MEMORY_WRITE_DONE:
		// Wait for confirmation from user interface loop
		if(transferComplete){memoryState = MEMORY_READY;}
		break;
	/* 		MEMORY READ STATES			*/
	case MEMORY_WAITING_FOR_READ:
		if(!isBusy()){memoryState = MEMORY_READ_IN_PROGRESS;}
		break;
	case MEMORY_READ_IN_PROGRESS:
		if(readPageDma()){
			if(!isBusy()){memoryState = MEMORY_READ_DONE;}
		}
		else{memoryState = MEMORY_WAITING_FOR_READ;}
		break;
	case MEMORY_READ_DONE:
		if(transferComplete){memoryState = MEMORY_READY;}
		break;
	}


	return memoryState;
}
