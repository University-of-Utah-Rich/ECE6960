#include <Arduino.h>
#include <RGBmatrixPanel.h>
#include <HardwareSerial.h>

// Most of the signal pins are configurable, but the CLK pin has some
// special constraints.  On 8-bit AVR boards it must be on PORTB...
// Pin 11 works on the Arduino Mega.  On 32-bit SAMD boards it must be
// on the same PORT as the RGB data pins (D2-D7)...
// Pin 8 works on the Adafruit Metro M0 or Arduino Zero,
// Pin A4 works on the Adafruit Metro M4 (if using the Adafruit RGB
// Matrix Shield, cut trace between CLK pads and run a wire to A4).

#define CLK 8 // USE THIS ON ADAFRUIT METRO M0, etc.
// #define CLK A4 // USE THIS ON METRO M4 (not M0)
// #define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE 9
#define LAT 10
#define A A0
#define B A1
#define C A2
#define D A3

// #define START_OF_FRAME 0xFFFF
// #define END_OF_FRAME 0xFFAA
#define MATRIX_WIDTH 32
#define MATRIX_HEIGHT 32
#define BAUD_RATE 9600

byte debugRead();

const int dataLength = 4 + 4 + 2 + 4; // fixed length of header and footer
const int MAX_IMAGE_SIZE = 100; // assuming a maximum image size of 100 bytes
byte incomingData[dataLength +
                  MAX_IMAGE_SIZE]; // buffer to store incoming packet
int dataIndex = 0;                 // index to store incoming bytes
unsigned int imageSize = 0;        // size of the image in bytes
unsigned int imageStart =
    0;                     // index of the start of the image data in the buffer
unsigned int checksum = 0; // checksum of the image bytes
RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE,
                      false); // create a new RGBmatrixPanel object
bool imageReceived = false;   // flag to indicate if an image has been received

void setup() {
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.fillScreen(matrix.Color888(0, 0, 0)); // clear the screen
  Serial.begin(
      BAUD_RATE); // start serial communication with a baud rate of BAUD_RATE
}

void loop() {
  while(!Serial || !Serial.available()) {
    Serial.println("Waiting for serial connection...");
  }
  if (Serial.available() > 0) { // check if there is incoming data
    incomingData[dataIndex++] =
        debugRead(); // store the incoming byte in the buffer

    if (dataIndex == 8) { // check if the start of frame has been received
      // check if the start of frame is correct
      if (incomingData[0] == 0xFF && incomingData[1] == 0xAA &&
          incomingData[2] == 0xAA && incomingData[3] == 0xFF) {
        // extract the image size from the incoming data
        imageSize = incomingData[7] | (incomingData[6] << 8) |
                    (incomingData[5] << 16) | (incomingData[4] << 24);
        // set the index for the start of the image data
        imageStart = dataIndex;
      } else {
        // reset the index if the start of frame is incorrect
        dataIndex = 0;
      }
    } else if (dataIndex ==
               imageStart + imageSize + 6) { // check if the image data and
                                             // checksum have been received
      // calculate the checksum of the image data
      for (int i = imageStart; i < imageStart + imageSize; i++) {
        checksum += incomingData[i];
      }
      // check if the received checksum matches the calculated checksum
      if (incomingData[imageStart + imageSize] == (checksum & 0xFF) &&
          incomingData[imageStart + imageSize + 1] ==
              ((checksum >> 8) & 0xFF)) {
        // check if the end of frame is correct
        if (incomingData[imageStart + imageSize + 2] == 0xAA &&
            incomingData[imageStart + imageSize + 3] == 0xFF &&
            incomingData[imageStart + imageSize + 4] == 0xFF &&
            incomingData[imageStart + imageSize + 5] == 0xAA) {
          // process the received image data here
        }
      }
      // reset the index for the next packet
      dataIndex = 0;
    }
  } else if (!imageReceived) {
    // draw a test image if no image has been received
    matrix.fillScreen(matrix.Color888(0, 0, 0)); // clear the screen
    matrix.setCursor(0, 0);
    matrix.setTextColor(matrix.Color888(255, 255, 255));
    matrix.print("No image received");
    delay(1000);
  }
}

/**
 * @brief Read a byte from the serial port and print it to the serial port
 * For debugging purposes.
 * @return byte The byte read from the serial port
 */
byte debugRead() {
  byte b;
  while (!Serial.available()) {
    // wait for data
  }

  b = Serial.read();
  Serial.print(b, HEX);
  return b;
}