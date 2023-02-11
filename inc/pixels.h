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
#include <arm_math.h>
#include <stdint.h>
#define SCAN_RATE (15)
extern uint8_t *image_buffer;
/**
 * @brief Set the pixel at position (x, y) to the given color
 * 
 * @param x The x position of the pixel
 * @param y The y position of the pixel
 * @param r The red component of the pixel
 * @param g The green component of the pixel
 * @param b The blue component of the pixel
 */
static void set_pixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set the pixel at position (x, y) to the given color using 32 bit
 * 
 * @param x The x position of the pixel
 * @param y The y position of the pixel
 * @param r The red component of the pixel
 * @param g The green component of the pixel
 * @param b The blue component of the pixel
 */
static void set_pixels_u32(uint16_t x, uint16_t y, uint32_t r, uint32_t g,
                           uint32_t b);