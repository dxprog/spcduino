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

#define BYTE_LEN 8

// Convenience arrays for looping over data pins. Super lazy stuff
const byte PINS_READ[BYTE_LEN]  = { D7, D6, D5, D4, D3, D2, D1, D0 };
const byte PINS_WRITE[BYTE_LEN] = { D0, D1, D2, D3, D4, D5, D6, D7 };

long cycle = 0;

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
  digitalWrite(RESET, LOW);

  Serial.begin(115200);
}

void changeDataDirection(byte direction) {
  pinMode(D0, direction);
  pinMode(D1, direction);
  pinMode(D2, direction);
  pinMode(D3, direction);
  pinMode(D4, direction);
  pinMode(D5, direction);
  pinMode(D6, direction);
  pinMode(D7, direction);
}

byte readData(byte addr) {
  changeDataDirection(INPUT);
  digitalWrite(PA0, addr & 0x1);
  digitalWrite(PA1, (addr >> 1) & 0x1);
  digitalWrite(READ, LOW);
  delay(1);

  byte retVal = 0;
  for (byte i = 0; i < BYTE_LEN; i++) {
    retVal = (retVal << 1) | digitalRead(PINS_READ[i]);
  }
  digitalWrite(READ, HIGH);
  delay(1);

  return retVal;
}

void writeData(byte addr, byte data) {
  changeDataDirection(OUTPUT);

  digitalWrite(PA0, addr & 0x1);
  digitalWrite(PA1, (addr >> 1) & 0x1);

  byte retVal = 0;
  for (byte i = 0; i < BYTE_LEN; i++) {
    digitalWrite(PINS_WRITE[i], (data >> i) & 0x1);
  }
  digitalWrite(WRITE, LOW);
  delay(1);
  digitalWrite(WRITE, HIGH);
  delay(1);
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

void loop() {
  if (cycle == 0) {
    digitalWrite(RESET, HIGH);
    delay(16);
    debugPorts();
    writeData(0x0, 0xCC);
    debugPorts();
  }
  cycle++;
}
