/*
 * memory.hpp
 *
 *  Created on: Dec 26, 2019
 *      Author: delta
 */

#ifndef MEMORY_HPP_
#define MEMORY_HPP_

#include "stm32f4xx_hal.h"
#include "scene.hpp"

const uint16_t FLASH_PAGE_SIZE = 256;

enum STATUS_REGISTER_1_BIT{
	SR1_BIT_BUSY = (1 << 0),
	SR1_BIT_WEL = (1 << 1),
	SR1_BIT_BLOCK_PROTECT_0 = (1 << 2),
	SR1_BIT_BLOCK_PROTECT_1 = (1 << 3),
	SR1_BIT_BLOCK_PROTECT_2 = (1 << 4),
	SR1_BIT_TOP_BOTTOM_PROTECT = (1 << 5),
	SR1_BIT_SECTOR_PROTECT = (1 << 6),
	SR1_BIT_STATUS_REGISTER_PROTECT = (1 << 7),
};
enum MEMORY_INSTRUCTION{
	WRITE_ENABLE = 0x06,
	VOLATILE_SR_WRITE_ENABLE = 0x50,
	WRITE_DISABLE = 0x04,

	READ_DATA = 0x03, // Followed by 3 address bytes, then chip reads back memory
	FAST_READ = 0x0B,
	PAGE_PROGRAM = 0x02, // Followed by 3 address bytes, and then the bytes to write to the page

	SECTOR_ERASE = 0x20,
	BLOCK_ERASE_32 = 0x52,
	BLOCK_ERASE_64 = 0xD8,
	CHIP_ERASE = 0x60,

	READ_STATUS_REGISTER_1 = 0x05,
	WRITE_STATUS_REGISTER_1 = 0x01,
	READ_STATUS_REGISTER_2 = 0x35,
	WRITE_STATUS_REGISTER_2 = 0x31,
	READ_STATUS_REGISTER_3 = 0x15,
	WRITE_STATUS_REGISTER_3 = 0x11,

	GLOBAL_BLOCK_LOCK = 0x7E,
	GLOBAL_BLOCK_UNLOCK = 0x98,
	READ_BLOCK_LOCK = 0x3D,
	INDIVIDUAL_BLOCK_LOCK = 0x36,
	INDIVIDUAL_BLOCK_UNLOCK = 0x39,

	ERASE_PROGRAM_SUSPEND = 0x75,
	ERASE_PROGRAM_RESUME = 0x7A,
	POWER_DOWN = 0xB9,

	ENABLE_RESET = 0x66,
	RESET_DEVICE = 0x99
};

enum MEMORY_STATE{
	MEMORY_WRITE_WAITING_FOR_ERASE,
	MEMORY_WRITE_ERASE_IN_PROGRESS,
	MEMORY_WAITING_FOR_WRITE,
	MEMORY_WRITE_IN_PROGRESS,
	MEMORY_WRITE_DONE,

	MEMORY_ERASE_WAITING_FOR_ERASE,
	MEMORY_ERASE_IN_PROGRESS,
	MEMORY_ERASE_DONE,

	MEMORY_WAITING_FOR_READ,
	MEMORY_READ_IN_PROGRESS,
	MEMORY_READ_DONE,

	MEMORY_READY,
};

class Memory{
public:
	Memory(SPI_HandleTypeDef* hspi, GPIO_TypeDef * chipSelectPort, uint16_t chipSelectPin);
	~Memory(void);

	void sendInstruction(MEMORY_INSTRUCTION instruction, uint32_t address);
	bool isBusy(void);
	bool writeEnabled(void);
	bool readyForWrite(void);

	uint8_t readByte(uint32_t address);
	void readBlock(uint32_t blockAddress, uint16_t nBytes, void* data);

	void writeByte(uint32_t address, uint8_t byte);
	void writeBlock64(uint32_t blockAddress, uint16_t nBytes, void* data);

	//void writeSceneDma(Scene& scene);
	//void readSceneDma(Scene& scene);

	MEMORY_STATE stateMachine();
	void dmaTransferComplete(void);
	void completeTransfer(void);
	bool writePageDma(void);
	bool writeSceneDma(SceneData& sceneData);
	bool readPageDma(void);
	bool readSceneDma(SceneData& sceneData);

	bool busyBitState, writeEnableBitState, dmaTransferInProgress, transferComplete;
private:
	uint32_t dmaBlockAddress, dmaPageCounter, dmaPageAmount, dmaBytesRemainder;
	uint8_t* dmaWritePointer;


	MEMORY_STATE memoryState = MEMORY_READY;
	SPI_HandleTypeDef* spi;
	GPIO_TypeDef* CS_GPIO_Port;
	uint16_t CS_GPIO_Pin;

};

#endif /* MEMORY_HPP_ */
