#ifndef SPC_700_H
#define SPC_700_H

#define NOP __asm__ __volatile__ ("nop\n\t")

// Pin definitions
#define PIN_READ   A0
#define PIN_RESET  A1
#define PIN_WRITE  A2

// SPC ports
#define PORT_0    0
#define PORT_1    1
#define PORT_2    2
#define PORT_3    3

void spc_reset();
uint8_t spc_read(uint8_t addr);
void spc_write(uint8_t addr, uint8_t value);
void spc_write_chunk(uint8_t *data, uint16_t len);
void spc_zero_wait(uint8_t value);

#endif
