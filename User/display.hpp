/*
 * Copyright (c) 2014 by ELECTRONIC ASSEMBLY <technik@lcd-module.de>
 * EA DOG Graphic (ST7565R) software library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
#include <stm32f405xx.h>
#include <stm32f4xx_hal.h>
#include <main.h>
#include <stdio.h>
#include <cstring>
#include "scene.hpp"
#include "font_6x8.h"

#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#define DISP_RST_GPIO_Port SPI2_DISP_RST_GPIO_Port
#define DISP_RST_Pin SPI2_DISP_RST_Pin
#define DISP_A0_GPIO_Port SPI2_DISP_A0_GPIO_Port
#define DISP_A0_Pin SPI2_DISP_A0_Pin
#define DISP_CS_GPIO_Port SPI2_DISP_CS_GPIO_Port
#define DISP_CS_Pin SPI2_DISP_CS_Pin

#define DOGM128 1
#define DOGL128 2
#define DOGM132 3

#define VIEW_BOTTOM 0xC0
#define VIEW_TOP 	0xC8

enum GRID_TYPE{
	GRID_TYPE_INT,
	GRID_TYPE_UINT,
	GRID_TYPE_CHAR,
	GRID_TYPE_NOTE,
};

class Display
{
  public:
    void initialize     (SPI_HandleTypeDef* hspi, uint8_t type);
    void clear			(void);
    void contrast       (uint8_t contr);
	void view			(uint8_t direction);
	void string         (uint8_t column, uint8_t page, const uint8_t *font_adress, const char *str);
	void string			(uint8_t column, uint8_t page, const uint8_t *font_adress, const char *str, bool invert);
	void rectangle		(uint8_t start_column, uint8_t start_page, uint8_t end_column, uint8_t end_page, uint8_t pattern);
	void picture		(uint8_t column, uint8_t page, const uint8_t *pic_adress);

	void stringToBuffer (uint8_t column, uint8_t page, const uint8_t *font_adress, const char *str, bool invert);

	void updateTopBar	(Scene& scene);
	void updateBottomBar(Scene& scene);

	void line(uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY);
	void box(uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY);

	void notification(const char* lineOne, const char* lineTwo);
	void progressBar(float progress, uint8_t page);
	void query(char* optionLeft, char* optionRight);

	void displayStepParameterGrid(STEP_PARAMETER parameter, uint8_t selection, Scene& scene);
	void laneParameters(Scene& scene);


	bool doneWriting(void);
	void clearBuffer(void);
	void writeDisplayBuffer	(void);
	bool writeDisplayBufferDMA (void);
	void dmaTransferComplete (void);

	uint8_t displayBuffer[8][128];
	void position   (uint8_t column, uint8_t page);

	bool dmaTransferInProgress = false;
  private:
    uint8_t p_cs;
    uint8_t p_si;
    uint8_t p_clk;
    uint8_t p_a0;
	uint8_t type;


	bool changedPage[8];

	SPI_HandleTypeDef* hspi;
	bool hardware;
    bool top_view;
	

    void command	(uint8_t dat);
    void data		(uint8_t dat);
    
    void spi_initialize	(uint8_t cs, uint8_t si, uint8_t clk);
    void spi_put_byte	(uint8_t dat);
    void spi_put		(uint8_t *dat, int len);
	void spi_out		(uint8_t dat);
};

#endif
