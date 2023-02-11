/**
 * @file pixels.c
 * @author Rich Baird (rich.baird@utah.edu)
 * @brief Functions for manipulating pixels. Declared in pixels.h
 * @version 0.1
 * @date 2023-02-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "pixels.h"

static void set_pixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t offset = y * 4 * 3;
	offset += x >> 3;
	// shift by the remainder times 4
	uint8_t shift = (x & 0x7u) << 2;
	uint32_t mask = ~(0xFu << shift);
	uint32_t pr = ((uint32_t)r & 0xFu) << shift;
	uint32_t pg = ((uint32_t)g & 0xFu) << shift;
	uint32_t pb = ((uint32_t)b & 0xFu) << shift;

	image_buffer[offset + 0] = (image_buffer[offset + 0] & mask) | pr;
	image_buffer[offset + 4] = (image_buffer[offset + 4] & mask) | pg;
	image_buffer[offset + 8] = (image_buffer[offset + 8] & mask) | pb;
}

static void set_pixels_u32(uint16_t x, uint16_t y, uint32_t r, uint32_t g, uint32_t b)
{
	uint16_t offset = y * 4 * 3;
	offset += x >> 3;
	image_buffer[offset + 0] = r;
	image_buffer[offset + 4] = g;
	image_buffer[offset + 8] = b;
}
