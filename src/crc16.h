#ifndef CRC16_H
#define CRC16_H

#define CCITT_POLY 0x1021

struct crc16 {
  // Current CRC value
  uint16_t crc = 0xFFFF;

  // Restart calculation
  void reset() { crc = 0xFFFF; }

  // Borrowed from avr-libc <util/crc16.h>
  // Performs CCITT CRC computation for one byte, MSB first
  uint16_t update(uint8_t data) {
    crc = crc ^ ((uint16_t)data << 8);
    for (int i = 0; i < 8; i++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ CCITT_POLY;
      else
        crc <<= 1;
    }
    return crc;
  }
};

#endif
