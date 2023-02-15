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

volatile uint8_t image_buffer[MAX_PIXELS * N_CHANNELS] = {0};
volatile uint8_t frame_buffer[MAX_PIXELS * N_CHANNELS] = {0};

/**
 * @brief Set the address pins, A, B, C, and D, based on the line number (0-15)
 *
 *
 * @param line The line number (0-15) of the LED matrix
 */
void set_pins_for_line(uint8_t line) {
  HAL_GPIO_WritePin(LINE_SELECT_PORT_A, LINE_SELECT_PIN_A,
                    (line & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LINE_SELECT_PORT_B, LINE_SELECT_PIN_B,
                    (line & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LINE_SELECT_PORT_C, LINE_SELECT_PIN_C,
                    (line & 0x03) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LINE_SELECT_PORT_D, LINE_SELECT_PIN_D,
                    (line & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void prepare_frame(uint8_t line) {
// Drive output enable (OE) high to disable the LED
  HAL_GPIO_WritePin(LED_OUTPUT_ENABLE_PORT, LED_OUTPUT_ENABLE_PIN,
                    GPIO_PIN_SET);

  // Drive GPIO-pins for line selection
  set_pins_for_line(line);

  // Latch data in the shift registers to the outputs by pulling STB low
  HAL_GPIO_WritePin(SHIFT_REGISTER_STB_PORT, SHIFT_REGISTER_STB_PIN,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SHIFT_REGISTER_STB_PORT, SHIFT_REGISTER_STB_PIN,
                    GPIO_PIN_SET);

  // Drive output enable (OE) low to enable the LED
  HAL_GPIO_WritePin(LED_OUTPUT_ENABLE_PORT, LED_OUTPUT_ENABLE_PIN,
                    GPIO_PIN_RESET);
}

void reset_frame(void) {
  // Drive output enable (OE) high to disable the LED
  HAL_GPIO_WritePin(LED_OUTPUT_ENABLE_PORT, LED_OUTPUT_ENABLE_PIN,
                    GPIO_PIN_SET);

  // Drive GPIO-pins for line selection
  set_pins_for_line(0);

  // Latch data in the shift registers to the outputs by pulling STB low
  HAL_GPIO_WritePin(SHIFT_REGISTER_STB_PORT, SHIFT_REGISTER_STB_PIN,
                    GPIO_PIN_RESET);
}

void frame_scan(uint8_t *frame_buffer) {
  static uint8_t line = 0;

  
// Get the brightness value for the current pixel
uint32_t brightness = 0;
// TODO: This isn't working for NUM_INTENSITY_LEVELS > 8
if (NUM_INTENSITY_LEVELS < 8) {
    brightness = frame_buffer[line] << (8 - NUM_INTENSITY_LEVELS);
} else if (NUM_INTENSITY_LEVELS > 8) {
    brightness = frame_buffer[line] >> (NUM_INTENSITY_LEVELS - 8);
} else {
    brightness = frame_buffer[line];
}

// Calculate the BAM binary sequence for the brightness value
  uint8_t mask = 0;
  
  for (int i = 0; i < NUM_INTENSITY_LEVELS; i++) {
    // Calculate the threshold value for the current cycle
    uint32_t threshold = (i + 1) * (UINT8_MAX / NUM_INTENSITY_LEVELS);

    // If the brightness value is greater than or equal to the threshold,
    // set the current bit in the binary sequence to 1
    if (brightness >= threshold) {
      mask |= 1 << i;
    }
  }

  // Start SPI transfer to upload next line into the shift registers
  HAL_SPI_Transmit_DMA(&hspi2, &mask, 1);

  // Increment line counter
  line = (line + 1) % NUM_LINES;
}

/**
 * @brief Convert a 24-bit RGB image to a frame buffer for the LED matrix
 * by converting each pixel to a 32-bit value modulated by the number of intensity levels
 * 
 * @param image The 24-bit RGB image
 * @param frame_buffer The frame buffer for the LED matrix
 * @param num_pixels The number of pixels in the image
 */
void convert_pixels_to_frame_buffer(uint8_t *image, uint32_t *frame_buffer,
                                   int num_pixels) {
  for (int i = 0; i < num_pixels; i++) {
    uint32_t pixel = image[i * 3];
    pixel |= (uint32_t)image[i * 3 + 1] << 8;
    pixel |= (uint32_t)image[i * 3 + 2] << 16;

    uint32_t value = 0;
    uint32_t level = 255 / (NUM_INTENSITY_LEVELS - 1);

    // Optimized loop to avoid unnecessary comparisons
    for (int j = 0; j < NUM_INTENSITY_LEVELS; j++) {        
      if (pixel >= j * level) {
        value |= 1 << j;
      } else {
        break;
      }
    }

    frame_buffer[i] = value;
  }
}
