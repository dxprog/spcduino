#define D0      2
#define D1      3
#define D2      4
#define D3      5
#define D4      6
#define D5      7
#define D6      8
#define D7      9
#define PA0    10
#define PA1    11
#define PA6    12
#define PA7    13
#define READ   A0
#define RESET  A1
#define WRITE  A2

#define PORT0    0
#define PORT1    1
#define PORT2    2
#define PORT3    3
#define BYTE_LEN 8

#define NOP __asm__ __volatile__ ("nop\n\t")

// Convenience arrays for looping over data pins. Super lazy stuff
const byte PINS_READ[BYTE_LEN]  = { D7, D6, D5, D4, D3, D2, D1, D0 };
const byte PINS_WRITE[BYTE_LEN] = { D0, D1, D2, D3, D4, D5, D6, D7 };

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

  0x8F, 0x05, 0xFC, //      Mov [0FCh], #timer_2
  0x8F, 0x80, 0xFB, //      Mov [0FBh], #timer_1
  0x8F, 0x27, 0xFA, //      Mov [0FAh], #timer_0

  0xCD, 0xCA,       //      Mov X, #stack_pointer
  0xBD,             //      Mov SP, X

  0x2F, 0xAB,       //        Bra 0FFC9h  ;Right when IPL puts AA-BB on the IO ports and waits for CC.
};

#define STATE_INIT  0
#define STATE_READY 1
#define STATE_TXFR  2

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
  pinMode(PA6, OUTPUT);
  pinMode(PA7, OUTPUT);

  digitalWrite(PA6, HIGH);
  digitalWrite(PA7, LOW);

  Serial.flush();
  Serial.begin(148986);
  Serial.println("Begin");
}

void changeDataDirection(byte direction) {
  DDRD = (DDRD & 0x03) | (direction == OUTPUT ? 0xFC : 0x00);
  DDRB = (DDRB & 0xFC) | (direction == OUTPUT ? 0x03 : 0x00);
}

byte readData(byte addr) {
  // Set the address
  PORTB = (PORTB & B11110011) | ((addr & 0x03) << 2);
  digitalWrite(READ, LOW);
  NOP;

  // Read the data straight off the registers
  changeDataDirection(INPUT);
  unsigned char retVal = ((PINB & 0x03) << 6) | ((PIND & 0xFC) >> 2);

  changeDataDirection(OUTPUT);
  digitalWrite(READ, HIGH);

  return retVal;
}

void writeBus(byte value) {
  digitalWrite(WRITE, value);
}

void writeDataRaw(byte addr, byte data) {
  changeDataDirection(OUTPUT);

  // Set the address and the upper two bits of data
  PORTB = (PORTB & 0xF0) | ((addr & 0x03) << 2) | ((data & 0xC0) >> 6);

  // Lower six bits
  PORTD = (PORTD & 0x03) | ((data & 0x3F) << 2);
}

void writeData(byte addr, byte data) {
  writeDataRaw(addr, data);
  writeBus(LOW);
  NOP;
  writeBus(HIGH);
}

void waitForPortZero(unsigned char value) {
  while (readData(PORT0) != value);
}

void writePortZeroAndWait(unsigned char value) {
  writeData(PORT0, value);
  waitForPortZero(value);
}

void debugPorts() {
  byte value = readData(0x0);
  Serial.print("Value on PORT0: ");
  Serial.println(value, HEX);
  value = readData(0x1);
  Serial.print("Value on PORT1: ");
  Serial.println(value, HEX);
  value = readData(0x2);
  Serial.print("Value on PORT2: ");
  Serial.println(value, HEX);
  value = readData(0x3);
  Serial.print("Value on PORT3: ");
  Serial.println(value, HEX);
}

void resetSPC() {
  digitalWrite(RESET, LOW);
  delay(1);
  digitalWrite(RESET, HIGH);
  byte port0 = 0;
  byte port1 = 0;
  while (port0 != 0xAA && port1 != 0xBB) {
    port0 = readData(PORT0);
    port1 = readData(PORT1);
  }
  Serial.println("SPC reset!");
}

void writeChunk(byte *data, unsigned short len) {
  for (int i = 0; i < len; i++) {
    writeData(PORT1, data[i]);
    writeData(PORT0, i);
    while (readData(PORT0) != i);
  }
}

void readSerialBuffer(byte *buffer, unsigned short len) {
  int bufferPos = 0;
  while (bufferPos < len) {
    while (Serial.available() < 32);
    for (int i = 0; i < 32; i++) {
      buffer[bufferPos] = Serial.read();
      bufferPos++;
    }
  }

  Serial.print("Read 0x");
  Serial.print(bufferPos, HEX);
  Serial.println(" bytes into buffer");
}

void initDsp() {
  // Receive the DSP data
  byte buffer[128];
  Serial.println("Receiving DSP data");
  readSerialBuffer(buffer, 128);

  // Write the DSP loader
  writeData(PORT2, 2);
  writeData(PORT3, 0);
  writeData(PORT1, 1);
  writePortZeroAndWait(0xCC);
  writeChunk(dspLoader, 28);

  Serial.println("DSP loader written");

  // Bit twiddling
  buffer[0x4C] = 0;
  buffer[0x6C] = 0x60;

  // Send the DSP data
  writeData(PORT2, 2);
  writeData(PORT3, 0);
  writeData(PORT1, 0);
  writePortZeroAndWait(sizeof(dspLoader) + 1);
  writeChunk(buffer, 127);

  // Write the last byte and wait for IPL ROM to cycle back around
  writeData(PORT1, buffer[127]);
  writeData(PORT0, 127);
  waitForPortZero(0xAA);

  Serial.println("DSP data written");
}

void loop() {
  if (state == STATE_INIT) {
    resetSPC();
    state = STATE_READY;
  } else if (state == STATE_READY && Serial.available()) {
    initDsp();
    state = STATE_TXFR;
  } else if (state == STATE_TXFR) {
    unsigned char buffer[0xFF];
    Serial.println("Receiving SPC data (SEND)");
    readSerialBuffer(buffer, 0x100);

    // Write page 0 of SPC data
    writeData(PORT2, 2);
    writeData(PORT3, 0);
    writeData(PORT1, 1);
    writePortZeroAndWait(0xCC);

    // Leave the first two bytes alone and also don't touch the register addresses
    Serial.println("Sending zero page data");
    for (int i = 2; i < 0xF0; i++) {
      writeData(PORT1, buffer[i]);
      writePortZeroAndWait(i - 2);
    }

    Serial.println("Zero page data written (SEND)");

    writeData(PORT1, 1);
    writeData(PORT2, 0);
    writeData(PORT3, 1);
    unsigned char port0Check = readData(PORT0) + 2;
    writePortZeroAndWait(port0Check);

    unsigned short checksum = 0;
    unsigned short currentAddr = 0x100;
    unsigned long start = micros();
    while (currentAddr > 0) {
      while (Serial.available() == 0);
      unsigned char data = Serial.read();
      writeData(PORT1, data);
      writePortZeroAndWait(currentAddr & 0xFF);

      checksum = (checksum + data) & 0xFF;

      currentAddr++;
    }

    Serial.println(micros() - start);

    Serial.println("SPC data written. Starting playback");
    writeData(PORT2, 0xC0);
    writeData(PORT3, 0xFF);
    writeData(PORT1, 0);
    port0Check = readData(PORT0) + 2;
    writePortZeroAndWait(port0Check);
    Serial.println("I've done all I can, boss");

    unsigned short bail = 512;
    while (readData(PORT0) != 0x53 && --bail > 0);

    if (bail == 0) {
      Serial.println("Go to bed");
      Serial.println(checksum);
      debugPorts();
    }

    writeData(PORT0, 0x00);
    writeData(PORT1, 0x00);
    writeData(PORT2, 0x72);
    writeData(PORT3, 0x04);

    state = 100;
  }
}
