#include "spc700.h"

#define READ   A0
#define RESET  A1
#define WRITE  A2
#define PA0    10
#define PA1    11

#define NOP __asm__ __volatile__ ("nop\n\t")

// Commands that can be issued by spc-player
#define CMD_RESET        1
#define CMD_LOAD_DSP     2
#define CMD_WRITE_CHUNK  3
#define CMD_PLAY         4

// Responses to send back to spc-player
#define RSP_OKAY         1
#define RSP_FAIL         2
#define RSP_BAD_CHECKSUM 3
#define RSP_READY        86

#define STATE_INIT  0
#define STATE_READY 1
#define STATE_TXFR  2

#define DSP_LOADER_SIZE   28
#define DSP_DATA_SIZE     128

byte state = STATE_INIT;

void setup() {
  pinMode(READ, OUTPUT);
  pinMode(WRITE, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(PA0, OUTPUT);
  pinMode(PA1, OUTPUT);

  Serial.begin(115200);
  Serial.write(RSP_READY);
}

/**
 * Reads len amount of bytes off the serial port
 *
 * @param {uint8_t} buffer Pointer to the buffer to write the serial data to
 * @param {uint16_t} len The amount of bytes to read
 * @return {bool} If the transfer went okay
 */
uint8_t read_serial_buffer(uint8_t *buffer, uint16_t len) {
  uint16_t bufferPos = 0;
  uint8_t checksum = 0;
  while (bufferPos < len) {
    while (Serial.available() == 0);
    buffer[bufferPos] = Serial.read();
    checksum = (checksum + buffer[bufferPos]) & 0xFF;
    bufferPos++;
  }

  // The last byte should be the expected checksum
  while (Serial.available() == 0);
  uint8_t expectedChecksum = Serial.read();

  // If the checksum doesn't _check out_, send a bad checksum response
  if (checksum != expectedChecksum) {
    Serial.write(RSP_BAD_CHECKSUM);
  }

  return checksum == expectedChecksum;
}

/**
 * Resets the SPC
 */
void cmd_reset() {
  spc_reset();
  Serial.write(RSP_OKAY);
}

/**
 * Reads the DSP loader program and register data from serial
 * and then writes it to the SPC
 *
 * @param {uint8_t[28]} loaderProgram The DSP loader program
 * @param {uint8_t[128]} dspRegisters The DSP register data
 */
void cmd_load_dsp() {
  // Receive the DSP loader
  uint8_t loaderProgram[28];
  uint8_t expectedChecksum;
  if (!read_serial_buffer(loaderProgram, 28)) {
    return;
  }

  // Send success
  Serial.write(RSP_OKAY);

  // Receive the DSP data
  uint8_t dspRegisters[128];
  if (!read_serial_buffer(dspRegisters, 128)) {
    return;
  }

  // Write the DSP loader
  spc_begin_transfer(0x0002);
  spc_write_chunk(loaderProgram, 28);

  // End the transfer and run the DSP loader
  spc_end_transfer(0x0002);

  // Now write the DSP register data
  spc_write_chunk(dspRegisters, 128);

  // Wait for the loader to finish and IPL ROM to kick back in
  spc_zero_wait(0xAA);

  Serial.write(RSP_OKAY);
}

void loop() {

  // Wait for a command to show up on serial
  if (Serial.available() > 0) {
    uint8_t command = Serial.read();

    switch (command) {
      case CMD_RESET:
        cmd_reset();
        break;
      case CMD_LOAD_DSP:
        cmd_load_dsp();
        break;
    }
  }

  /*
  if (state == STATE_INIT) {
    spc_reset();
    state = STATE_READY;
  } else if (state == STATE_READY && Serial.available()) {
    initDsp();
    state = STATE_TXFR;
  } else if (state == STATE_TXFR) {
    unsigned char buffer[0xFF];
    Serial.println("Receiving zero page data (SEND)");
    readSerialBuffer(buffer, 0x100);

    // Write page 0 of SPC data
    spc_begin_transfer(0x0002);

    // Leave the first two bytes alone and also don't touch the register addresses
    unsigned short checksum = 0;
    Serial.println("Sending zero page data");
    for (int i = 2; i < 0xF0; i++) {
      spc_write(PORT_1, buffer[i]);
      spc_write(PORT_0, i - 2);
      spc_zero_wait(i - 2);
    }

    Serial.println("Zero page data written");
    Serial.println("Receiving SPC data (SEND)");

    spc_begin_chunk(0x0100);

    checksum = 0;
    unsigned short currentAddr = 0x100;
    unsigned long start = micros();
    uint8_t port0Check = 0;
    while (currentAddr > 0) {
      while (Serial.available() == 0);
      unsigned char data = Serial.read();
      port0Check = currentAddr & 0xFF;
      spc_write(PORT_1, data);
      spc_write(PORT_0, port0Check);
      spc_zero_wait(port0Check);

      checksum = (checksum + data) & 0xFF;

      currentAddr++;
    }

    Serial.println(micros() - start);

    Serial.print("SPC data written (");
    Serial.print(checksum);
    Serial.println("). Starting playback");
    spc_end_transfer(0xE6D0);
    Serial.println("I've done all I can, boss");

    spc_write(PORT_3, 0x01);
    spc_write(PORT_0, 0x01);

    unsigned short bail = 512;
    while (spc_read(PORT_0) != 0x53 && --bail > 0);

    spc_write(PORT_0, 0x00);
    spc_write(PORT_1, 0x00);
    spc_write(PORT_2, 0x00);
    spc_write(PORT_3, 0x00);

    if (bail == 0) {
      Serial.println("Go to bed");
      Serial.println(checksum);
    }
    state = 100;
  }
  */
}
