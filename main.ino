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
#define CMD_START_SPC    3
#define CMD_SPC_CHUNK    4
#define CMD_PLAY         5

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
#define ZERO_PAGE_SIZE    237 // 0xEF - 2

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

/**
 * Begins transferring the program data to the SPC with zero page data
 *
 * @param {uint8_t[ZERO_PAGE_SIZE]} zeroPage The zero page data
 */
void cmd_start_spc() {
  // We're going to get zero page data, so retrive that first
  uint8_t zeroPage[ZERO_PAGE_SIZE];
  if (!read_serial_buffer(zeroPage, ZERO_PAGE_SIZE)) {
    return;
  }

  // Start a new transfer
  spc_begin_transfer(0x0002);
  spc_write_chunk(zeroPage, ZERO_PAGE_SIZE);
  Serial.write(RSP_OKAY);
}

/**
 * Uses an already started transfer to send data to the SPC
 * starting at the specified address
 *
 * @param {uint16_t} addr Little endian encoded address to start transferring at
 * @param {uint8_t} len The length of data to transfer
 * @param {uint8_t *} data The data to transfer
 */
void cmd_spc_chunk() {
  // Read in the transfer parameters
  uint8_t params[3];
  if (!read_serial_buffer(params, 3)) {
    return;
  }
  Serial.write(RSP_OKAY);

  uint16_t addr = (params[1] << 8) | params[0];
  uint8_t len = params[2];

  // If the data is zero length, bail
  if (len == 0) {
    return;
  }

  // Begin transferring the data from serial
  uint8_t buffer[len];
  if (!read_serial_buffer(buffer, len)) {
    return;
  }

  // And send it off to the SPC
  spc_begin_chunk(addr);
  spc_write_chunk(buffer, len);
  Serial.write(RSP_OKAY);
}

/**
 * Begins playback of the SPC
 *
 * @param {uint16_t} bootAddr Little endian encoded address of the program boot loader
 * @param {uint8_t[4]} portValues The values to reset ports 0 - 3 to
 */
void cmd_play() {
  uint8_t params[6];
  if (!read_serial_buffer(params, 6)) {
    return;
  }

  bool arePortsZero = !params[2] && !params[3] && !params[4] && !params[5];

  // End the transfer and JMP to the boot loader
  spc_end_transfer((params[1] << 8) | params[0]);

  // If all the ports were zero, write these values out before waiting for the boot code.
  // TODO: Actually understand what this is about
  if (arePortsZero) {
    spc_write(PORT_3, 1);
    spc_write(PORT_0, 1);
  }

  // Wait for the boot good code
  spc_zero_wait(0x53);

  // Restore the original port values
  spc_write(PORT_0, params[2]);
  spc_write(PORT_1, params[3]);
  spc_write(PORT_2, params[4]);
  spc_write(PORT_3, params[5]);

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
      case CMD_START_SPC:
        cmd_start_spc();
        break;
      case CMD_SPC_CHUNK:
        cmd_spc_chunk();
        break;
      case CMD_PLAY:
        cmd_play();
        break;
    }
  }

}
