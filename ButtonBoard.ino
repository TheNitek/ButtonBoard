#include <Arduino.h>
#include <Wire.h>
#include <SparkFunSX1509.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>


// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io; // Create an SX1509 object to be used throughout

// SX1509 Pins:
const byte SX1509_BUTTON_PIN = 0; // Active-low button
const byte SX1509_INPUT_PIN = 8; // Floating or jumpered input

#define BUTTON_COUNT 9

#define VOLUME_PIN A0

#define VS1053_CS      16     // VS1053 chip select pin (output)
#define VS1053_DCS     15     // VS1053 Data/command select pin (output)
#define CARDCS          2     // Card chip select pin
#define VS1053_DREQ     0     // VS1053 Data request, ideally an Interrupt pin
#define VS1053_RESET   -1

Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);

int8_t currentFile = -1;
int8_t buttonPressed = -1;

const uint16_t maxVolume = 1024;
const float volumeStep = (float)50/maxVolume;
int8_t currentVolume = -1;

void setup() {
  Serial.begin(115200);

  if (!io.begin(SX1509_ADDRESS)) {
    Serial.println("Failed to communicate with IO extender");
    while (1);
  }

  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }

  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);

  Serial.println(F("VS1053 found"));
 
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1);  // don't do anything more
  }
  Serial.println("SD OK!");

  pinMode(VOLUME_PIN, INPUT);

  for(uint8_t i = 0; i < BUTTON_COUNT; i++) {
    io.pinMode(i, INPUT_PULLUP);
  }

}

void loop() {
  if(musicPlayer.playingMusic) {
    setVolume();
  }
  checkButtons();
}

void setVolume() {
  uint8_t v = 50-volumeStep*analogRead(VOLUME_PIN);
  int8_t volumeDiff = currentVolume - v;
  if(currentVolume < 0 || abs(volumeDiff) > 1) {
    Serial.printf("Volume: %d\n", v);
    musicPlayer.setVolume(v, v);
    currentVolume = v;
  }
}

void checkButtons() {
  for(uint8_t i = 0; i < BUTTON_COUNT; i++) {
    if(io.digitalRead(i) == HIGH) {
      continue;
    }

    if(buttonPressed == i) {
      return;
    }
    buttonPressed = i;

    if(musicPlayer.playingMusic) {
      musicPlayer.stopPlaying();
      if(currentFile == i) {
        currentFile = -1;
        return;
      }
    }

    currentFile = i;
    char file[10] = "";
    snprintf(file, sizeof(file), "%d.mp3", currentFile);
    Serial.println(file);
    musicPlayer.startPlayingFile(file);
    return;
  }
  buttonPressed = -1;
}