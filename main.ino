#include "spc700.h"

#define READ   A0
#define RESET  A1
#define WRITE  A2
#define PA0    10
#define PA1    11

#define NOP __asm__ __volatile__ ("nop\n\t")

// Commands that can be issued by spc-player
#define CMD_RESET        0
#define CMD_LOAD_DSP     1
#define CMD_WRITE_CHUNK  2
#define CMD_PLAY         3

// Responses to send back to spc-player
#define RSP_OKAY         0
#define RSP_FAIL         1

#define STATE_INIT  0
#define STATE_READY 1
#define STATE_TXFR  2

// DSP boot loader
static byte dspLoader[] =
{ //For loading the 128 byte DSP ram. DO NOT CHANGE.
  0xC4, 0xF2,       //START:  Mov [0F2h], A
  0x64, 0xF4,       //LOOP:   Cmp A, [0F4h]
  0xD0, 0xFC,       //        Bne LOOP
  0xFA, 0xF5, 0xF3, //        Mov [0F3h], [0F5h]
  0xC4, 0xF4,       //        Mov [0F4h], A
  0xBC,             //        Inc A
  0x10, 0xF2,       //        Bpl START

  0x8F, 0xFF, 0xFC, //      Mov [0FCh], #timer_2
  0x8F, 0xFF, 0xFB, //      Mov [0FBh], #timer_1
  0x8F, 0x10, 0xFA, //      Mov [0FAh], #timer_0

  0xCD, 0xF9,       //      Mov X, #stack_pointer
  0xBD,             //      Mov SP, X

  0x2F, 0xAB,       //        Bra 0FFC9h  ;Right when IPL puts AA-BB on the IO ports and waits for CC.
};

byte state = STATE_INIT;
byte lastPort0 = 0;
byte currentAddress = 0;
byte currentDirection = 2;

void setup() {
  pinMode(READ, OUTPUT);
  pinMode(WRITE, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(PA0, OUTPUT);
  pinMode(PA1, OUTPUT);

  Serial.flush();
  Serial.begin(148986);
  Serial.println("Begin");
}

void readSerialBuffer(byte *buffer, unsigned short len) {
  int bufferPos = 0;
  int checksum = 0;
  while (bufferPos < len) {
    while (Serial.available() < 32);
    for (int i = 0; i < 32; i++) {
      buffer[bufferPos] = Serial.read();
      checksum = (checksum + buffer[bufferPos]) & 0xFF;
      bufferPos++;
    }
  }

  Serial.print("Read 0x");
  Serial.print(bufferPos, HEX);
  Serial.print(" bytes into buffer with a checksum: ");
  Serial.println(checksum);
}

void initDsp() {
  // Receive the DSP data
  byte buffer[128];
  Serial.println("Receiving DSP data");
  readSerialBuffer(buffer, 128);

  // Write the DSP loader
  spc_write(PORT_2, 0x02);
  spc_write(PORT_3, 0x00);
  spc_write(PORT_1, 0x01);
  spc_write(PORT_0, 0xCC);
  spc_zero_wait(0xCC);
  spc_write_chunk(dspLoader, 28);

  Serial.println("DSP loader written");

  // Bit twiddling
  buffer[0x4C] = 0x00;
  buffer[0x6C] = 0x60;

  // Send the DSP data
  spc_write(PORT_2, 0x02);
  spc_write(PORT_3, 0x00);
  spc_write(PORT_1, 0x00);
  spc_write(PORT_0, 29);
  spc_zero_wait(29);
  spc_write_chunk(buffer, 127);

  // Write the last byte and wait for IPL ROM to cycle back around
  spc_write(PORT_1, buffer[127]);
  spc_write(PORT_0, 127);
  spc_zero_wait(0xAA);

  Serial.println("DSP data written");
}

void loop() {
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
    spc_write(PORT_2, 2);
    spc_write(PORT_3, 0);
    spc_write(PORT_1, 1);
    spc_write(PORT_0, 0xCC);
    spc_zero_wait(0xCC);

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

    spc_write(PORT_1, 1);
    spc_write(PORT_2, 0);
    spc_write(PORT_3, 1);
    unsigned char port0Check = spc_read(PORT_0) + 2;
    spc_write(PORT_0, port0Check);
    spc_zero_wait(port0Check);

    checksum = 0;
    unsigned short currentAddr = 0x100;
    unsigned long start = micros();
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
    spc_write(PORT_2, 0x90);
    spc_write(PORT_3, 0xFF);
    spc_write(PORT_1, 0);
    port0Check = spc_read(PORT_0) + 2;
    spc_write(PORT_0, port0Check);
    spc_zero_wait(port0Check);
    Serial.println("I've done all I can, boss");

    unsigned short bail = 512;
    while (spc_read(PORT_0) != 0x53 && --bail > 0);

    spc_write(PORT_0, 0x73);
    spc_write(PORT_1, 0x00);
    spc_write(PORT_2, 0xFF);
    spc_write(PORT_3, 0x00);

    if (bail == 0) {
      Serial.println("Go to bed");
      Serial.println(checksum);
    }
    state = 100;
  }
}
