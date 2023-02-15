/**
 * @file pixels.h
 * @author Rich Baird (rich.baird@utah.edu)
 * @brief Function prototypes for pixels.c
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "main.h"
#include "spi.h"
#include <stdint.h>

#define LED_OUTPUT_ENABLE_PORT OE_GPIO_Port
#define LED_OUTPUT_ENABLE_PIN OE_Pin

#define LINE_SELECT_PORT_A A_GPIO_Port
#define LINE_SELECT_PIN_A A_Pin
#define LINE_SELECT_PORT_B B_GPIO_Port
#define LINE_SELECT_PIN_B B_Pin
#define LINE_SELECT_PORT_C C_GPIO_Port
#define LINE_SELECT_PIN_C C_Pin
#define LINE_SELECT_PORT_D D_GPIO_Port
#define LINE_SELECT_PIN_D D_Pin

#define SHIFT_REGISTER_STB_PORT STB_GPIO_Port
#define SHIFT_REGISTER_STB_PIN STB_Pin
#define MAX_PIXELS 1024
#define N_CHANNELS 3
#define NUM_LINES 16
#define NUM_INTENSITY_LEVELS 64

void convert_pixels_to_frame_buffer(uint8_t* image, uint32_t* frame_buffer, int num_pixels);
void frame_scan(uint8_t *frame_buffer);
volatile extern uint8_t image_buffer[MAX_PIXELS * N_CHANNELS];
volatile extern uint8_t frame_buffer[MAX_PIXELS * N_CHANNELS];
