#include "ADS131M04.h"
#include "Arduino.h"
#include "SPI.h"
#include "math.h"

// Init class
bool ADS131M04::begin(SPIClass* spi, SPISettings spi_settings, uint8_t cs_pin, uint8_t drdy_pin) {
  // Init SPI Interface
  this->spi = spi;
  this->spi->beginTransaction(spi_settings);

  // Save pin values
  CS_PIN = cs_pin;
  DRDY_PIN = drdy_pin;

  // Configure pin modes
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(DRDY_PIN, INPUT);

  // Reset ADC
  if (!resetSoft()) {
    return false;
  }

  // Check ID register
  return (readRegister(REG_ID) & (uint16_t)0xFF00) == (uint16_t)0x2400;
}

// Transfer one data word over SPI bus
// Expects data word to be right (LSB) aligned
// Assumes 2 bytes of relevant data since all commands/values are 16 bits
uint32_t ADS131M04::transfer_word(uint32_t data, int bytes) {
  uint32_t out = 0;
  uint8_t block, resp;

  // Make data MSB aligned
  data <<= 8 * (word_bytes - bytes);

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // May need to be handled differently for other data formats

  // Send word as 1-byte chunks
  for (int offset = 8 * (word_bytes - 1); offset >= 0; offset -= 8) {
    // Send byte
    block = (data & ((uint32_t)0xFF << offset)) >> offset;
    resp = spi->transfer(block);

    // Update CRC
    if (spi_crc_enabled) {
      tx_crc.update(block);
      rx_crc.update(resp);
    }

    // Save response
    out |= (uint32_t)resp << offset;
  }

  return out;
}

// Generic function for sending commands to the ADC
// Returns the ADC's initial response to the command
// Sends null words so that the caller can get the relevant response right away
uint16_t ADS131M04::send_command(uint16_t cmd, int null_words) {
  // Reset CRC at beginning of frame
  if (spi_crc_enabled) {
    tx_crc.reset();
    rx_crc.reset();
  }

  // Get response
  uint32_t resp = transfer_word(cmd);
  switch (word_length) {
    case WORD_LENGTH_24BITS:
    case WORD_LENGTH_32BITS_SGN_EXT:
      resp >>= 8;
      break;
    case WORD_LENGTH_32BITS_LSB_PAD:
      resp >>= 16;
      break;
  };

  // Send TX CRC word
  if (spi_crc_enabled && null_words > 0) {
    transfer_word(tx_crc.crc);
    null_words--;
  }

  // Transfer null words
  for (int i = 0; i < null_words; i++) {
    transfer_word(0);
  }

  return (uint16_t)resp;
}

// Return channel voltages
adcOutput ADS131M04::readADC(void) {
  adcOutput out;
  uint32_t raw_readings[4];

  // Pull chip select low
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  // Send null command
  send_command(CMD_NULL, 0);

  // Send CRC if enabled
  raw_readings[0] = transfer_word(spi_crc_enabled ? tx_crc.crc : 0);

  // Get other channel data
  for (int i = 1; i < 4; i++) {
    raw_readings[i] = transfer_word(0);
  }

  // Check CRC
  transfer_word(0);
  out.valid = !spi_crc_enabled || rx_crc.crc == 0;

  // Convert readings to signed integers
  for (int i = 0; i < 4; i++) {
    switch (word_length) {
      case WORD_LENGTH_16BITS:
        out.raw_values[i] = (int32_t)(raw_readings[i] << 16) >> 8;
        break;
      case WORD_LENGTH_24BITS:
      case WORD_LENGTH_32BITS_SGN_EXT:
        out.raw_values[i] = (int32_t)(raw_readings[i] << 8) >> 8;
        break;
      case WORD_LENGTH_32BITS_LSB_PAD:
        out.raw_values[i] = (int32_t)raw_readings[i] >> 8;
        break;
    };

    // Convert readings to voltages
    out.voltages[i] =
        out.raw_values[i] / (float)0x7FFFFF * (1.2 / pga_gains[i]);
  }

  // Pull chip select high
  delayMicroseconds(1);
  digitalWrite(CS_PIN, HIGH);

  return out;
}

// Single-register write operation
// Returns true if successful
bool ADS131M04::writeRegister(uint8_t address, uint16_t value) {
  // Pull chip select low
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  // Issue single-register write command
  uint16_t cmd = (CMD_WRITE_REG) | (address << 7) | 0;
  send_command(cmd, 0);

  // Transfer register value
  transfer_word(value);

  // Send CRC word
  if (spi_crc_enabled) {
    transfer_word(tx_crc.crc);
  } else {
    transfer_word(0);
  }

  // Send 3 additional null words
  for (int i = 0; i < 3; i++) {
    transfer_word(0);
  }

  // Send a null command so we can check the response
  uint16_t resp = send_command(CMD_NULL);

  // Response format: 010a aaaa ammm mmmm
  uint8_t resp_prefix = (resp & 0xE000) >> 13;
  uint8_t resp_addr = (resp & 0x1F80) >> 7;
  uint8_t resp_count = (resp & 0x007F) + 1;

  // Pull chip select high
  delayMicroseconds(1);
  digitalWrite(CS_PIN, HIGH);

  return (resp_prefix == 0b010) && (resp_addr == address) && (resp_count == 1);
}

// Modifies certain bits of a register by applying a mask
bool ADS131M04::writeRegisterMasked(uint8_t address, uint16_t value,
                                    uint16_t mask) {
  // Pull old register value
  uint16_t reg, old;
  reg = old = readRegister(address);

  // Clear bits using mask
  reg &= ~mask;

  // Set bits using value
  reg |= value;

  // Only write register if the value changed
  if (reg != old) {
    return writeRegister(address, reg);
  }
  return true;
}

// Read the value of a single register
// Returns right (LSB) aligned data
uint16_t ADS131M04::readRegister(uint8_t address) {
  // Pull chip select low
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  // Issue single-register read command
  uint16_t cmd = CMD_READ_REG | (address << 7 | 0);
  send_command(cmd);

  // Send null command to get register value
  uint16_t resp = send_command(CMD_NULL);

  // Pull chip select high
  delayMicroseconds(1);
  digitalWrite(CS_PIN, HIGH);

  return resp;
}

// Reset the ADC via software
// Returns 1 if reset was successful
bool ADS131M04::resetSoft(void) {
  // Pull chip select low
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  // Send reset command
  send_command(CMD_RESET);
  delay(1);  // 250us min?

  // Get response
  uint16_t resp = send_command(CMD_NULL);

  // Pull chip select high
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);

  // Check response
  if (resp == RSP_RESET_OK) {
    resetSettings();
    return true;
  }
  return false;
}

// Reset settings variables
void ADS131M04::resetSettings(void) {
  word_length = WORD_LENGTH_24BITS;
  word_bytes = 3;           // 24-bit words
  spi_crc_enabled = false;  // CRC disabled
  for (int i = 0; i < 4; i++) {
    pga_gains[i] = 1;
  }
}

// Enter standby mode
bool ADS131M04::enterStandby(void) {
  // Pull chip select low
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  // Send standby command
  send_command(CMD_STANDBY);

  // Send null command to check status
  uint16_t resp = send_command(CMD_NULL);

  // Pull chip select high
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);

  return resp == RSP_STANDBY_OK;
}

// Wake up from standby mode
bool ADS131M04::wakeFromStandby(void) {
  // Pull chip select low
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  // Send standby command
  send_command(CMD_WAKEUP);

  // Send null command to check status
  uint16_t resp = send_command(CMD_NULL);

  // Pull chip select high
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);

  return resp == RSP_WAKEUP_OK;
}

// Lock the SPI interface
bool ADS131M04::lockSPI(void) {
  // Pull chip select low
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  // Send standby command
  send_command(CMD_LOCK);

  // Send null command to check status
  uint16_t resp = send_command(CMD_NULL);

  // Pull chip select high
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);

  return resp == RSP_LOCK_OK;
}

// Unlock the SPI interface
bool ADS131M04::unlockSPI(void) {
  // Pull chip select low
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  // Send standby command
  send_command(CMD_UNLOCK);

  // Send null command to check status
  uint16_t resp = send_command(CMD_NULL);

  // Pull chip select high
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);

  return resp == RSP_UNLOCK_OK;
}

// Check the status register data ready bits
// The channel mask should be in the form 0bxxxx
bool ADS131M04::isDataReadySoft(uint16_t channel_mask) {
  // Check channel mask
  if (channel_mask & 0xFFF0) {
    return 0;
  }

  // Read register
  return (readRegister(REG_STATUS) & channel_mask) == channel_mask;
}

// Enable or disable CRC for SPI interface
bool ADS131M04::enableCRC(bool setting) {
  bool ret =
      writeRegisterMasked(REG_MODE, setting << 12, REGMASK_MODE_RX_CRC_EN);
  spi_crc_enabled = setting;
  return ret;
}

// Set data word format
// 00b = 16 bits
// 01b = 24 bits (default)
// 10b = 32 bits; LSB zero padding
// 11b = 32 bits; MSB sign extension
bool ADS131M04::setWordLength(uint8_t word_length) {
  if (word_length > 3) {
    return false;
  }
  bool out =
      writeRegisterMasked(REG_MODE, word_length << 8, REGMASK_MODE_WLENGTH);

  // Save setting
  this->word_length = word_length;
  switch (word_length) {
    case WORD_LENGTH_16BITS:
      word_bytes = 2;
      break;
    case WORD_LENGTH_24BITS:
      word_bytes = 3;
      break;
    case WORD_LENGTH_32BITS_LSB_PAD:
    case WORD_LENGTH_32BITS_SGN_EXT:
      word_bytes = 4;
      break;
  };

  return out;
}

// Select the source for the data ready pin
// 00b = Most lagging enabled channel (default)
// 01b = Logic OR of all the enabled channels
// 10b = Most leading enabled channel
// 11b = Most leading enabled channel
bool ADS131M04::setDrdySource(uint8_t drdySource) {
  return (drdySource <= 3) &&
         writeRegisterMasked(REG_MODE, drdySource << 2, REGMASK_MODE_DRDY_SEL);
}

// Set idle state of drdy pin
// 0 -> logic high (default)
// 1 -> high impedance
bool ADS131M04::setDrdyIdleState(uint8_t drdyState) {
  return (drdyState <= 1) &&
         writeRegisterMasked(REG_MODE, drdyState < 1, REGMASK_MODE_DRDY_HiZ);
}

// Set data ready format
// 0 -> logic low (default)
// 1 -> pulses
bool ADS131M04::setDrdyFormat(uint8_t drdyFormat) {
  return (drdyFormat <= 1) &&
         writeRegisterMasked(REG_MODE, drdyFormat, REGMASK_MODE_DRDY_FMT);
}

// Enable or disable a given channel
bool ADS131M04::setChannelEnable(uint8_t channel, bool enable) {
  switch (channel) {
    case 0:
      return writeRegisterMasked(REG_CLOCK, enable << 8, REGMASK_CLOCK_CH0_EN);
    case 1:
      return writeRegisterMasked(REG_CLOCK, enable << 9, REGMASK_CLOCK_CH1_EN);
    case 2:
      return writeRegisterMasked(REG_CLOCK, enable << 10, REGMASK_CLOCK_CH2_EN);
    case 3:
      return writeRegisterMasked(REG_CLOCK, enable << 11, REGMASK_CLOCK_CH3_EN);
  };

  return false;
}

// Set modulator oversampling ratio
// 1000b = 64,    64 kSPS
// 0000b = 128,   32 kSPS
// 0001b = 256,   16 kSPS
// 0010b = 512,   8 kSPS
// 0011b = 1024,  4 kSPS
// 0100b = 2048,  2 kSPS
// 0101b = 4096,  1 kSPS
// 0110b = 8192,  500 SPS
// 0111b = 16256, 250 SPS
bool ADS131M04::setOSR(uint16_t osr) {
  return (osr <= 7) &&
         writeRegisterMasked(REG_CLOCK, osr << 2, REGMASK_CLOCK_OSR);
}

// Set the power mode of the ADC
// 00b = Very-low-power
// 01b = Low-power
// 10b = High-resolution (default)
// 11b = High-resolution
bool ADS131M04::setPowerMode(uint8_t powerMode) {
  return (powerMode <= 3) &&
         writeRegisterMasked(REG_CLOCK, powerMode, REGMASK_CLOCK_PWR);
}

// Set PGA gain for a given channel
// 000b = 1 (default)
// 001b = 2
// 010b = 4
// 011b = 8
// 100b = 16
// 101b = 32
// 110b = 64
// 111b = 128
bool ADS131M04::setChannelPGA(uint8_t channel, uint16_t pga) {
  // Check/save gain
  if (pga > 7 || channel > 3) {
    return false;
  } else {
    pga_gains[channel] = pow(2, pga);
  }

  switch (channel) {
    case 0:
      return writeRegisterMasked(REG_GAIN, pga << 0, REGMASK_GAIN_PGAGAIN0);
    case 1:
      return writeRegisterMasked(REG_GAIN, pga << 4, REGMASK_GAIN_PGAGAIN1);
    case 2:
      return writeRegisterMasked(REG_GAIN, pga << 8, REGMASK_GAIN_PGAGAIN2);
    case 3:
      return writeRegisterMasked(REG_GAIN, pga << 12, REGMASK_GAIN_PGAGAIN3);
  };
  return false;
}

// Set global chop delay. Units = # clock periods
// 0000b = 2
// 0001b = 4
// 0010b = 8
// 0011b = 16 (default)
// 0100b = 32
// 0101b = 64
// 0110b = 128
// 0111b = 256
// 1000b = 512
// 1001b = 1024
// 1010b = 2048
// 1011b = 4096
// 1100b = 8192
// 1101b = 16384
// 1110b = 32768
// 1111b = 65536
bool ADS131M04::setGlobalChopDelay(uint8_t delay) {
  return (delay <= 15) &&
         writeRegisterMasked(REG_CFG, delay << 9, REGMASK_CFG_GC_DLY);
}

// Enable global chop
bool ADS131M04::enableGlobalChop(bool setting) {
  return writeRegisterMasked(REG_CFG, setting << 8, REGMASK_CFG_GC_EN);
}

// Set channel input MUX
// 00b = AIN0P and AIN0N (default)
// 01b = ADC inputs shorted
// 10b = Positive DC test signal
// 11b = Negative DC test signal
bool ADS131M04::setInputChannelSelection(uint8_t channel, uint8_t input) {
  if (input > 3 || channel > 3) {
    return false;
  }

  switch (channel) {
    case 0:
      return writeRegisterMasked(REG_CH0_CFG, input, REGMASK_CHX_CFG_MUX);
    case 1:
      return writeRegisterMasked(REG_CH1_CFG, input, REGMASK_CHX_CFG_MUX);
    case 2:
      return writeRegisterMasked(REG_CH2_CFG, input, REGMASK_CHX_CFG_MUX);
    case 3:
      return writeRegisterMasked(REG_CH3_CFG, input, REGMASK_CHX_CFG_MUX);
  };
  return false;
}

// Calibrate channel offset
bool ADS131M04::setChannelOffsetCalibration(uint8_t channel, int32_t offset) {
  if (channel > 3 || abs(offset) > 0x7FFFFF) {
    return false;
  }

  uint16_t MSB = offset >> 8;
  uint8_t LSB = offset;

  switch (channel) {
    case 0:
      return (writeRegisterMasked(REG_CH0_OCAL_MSB, MSB, 0xFFFF) &&
              writeRegisterMasked(REG_CH0_OCAL_LSB, LSB << 8,
                                  REGMASK_CHX_OCAL0_LSB));
    case 1:
      return (writeRegisterMasked(REG_CH1_OCAL_MSB, MSB, 0xFFFF) &&
              writeRegisterMasked(REG_CH1_OCAL_LSB, LSB << 8,
                                  REGMASK_CHX_OCAL0_LSB));
    case 2:
      return (writeRegisterMasked(REG_CH2_OCAL_MSB, MSB, 0xFFFF) &&
              writeRegisterMasked(REG_CH2_OCAL_LSB, LSB << 8,
                                  REGMASK_CHX_OCAL0_LSB));
    case 3:
      return (writeRegisterMasked(REG_CH3_OCAL_MSB, MSB, 0xFFFF) &&
              writeRegisterMasked(REG_CH3_OCAL_LSB, LSB << 8,
                                  REGMASK_CHX_OCAL0_LSB));
  };
  return false;
}

// Calibrate channel gain
bool ADS131M04::setChannelGainCalibration(uint8_t channel, uint32_t gain) {
  if (channel > 3 || gain > 0xFFFFFF) {
    return false;
  }

  uint16_t MSB = gain >> 8;
  uint8_t LSB = gain;

  switch (channel) {
    case 0:
      return (writeRegisterMasked(REG_CH0_GCAL_MSB, MSB, 0xFFFF) &&
              writeRegisterMasked(REG_CH0_GCAL_LSB, LSB << 8,
                                  REGMASK_CHX_GCAL0_LSB));
    case 1:
      return (writeRegisterMasked(REG_CH1_GCAL_MSB, MSB, 0xFFFF) &&
              writeRegisterMasked(REG_CH1_GCAL_LSB, LSB << 8,
                                  REGMASK_CHX_GCAL0_LSB));
    case 2:
      return (writeRegisterMasked(REG_CH2_GCAL_MSB, MSB, 0xFFFF) &&
              writeRegisterMasked(REG_CH2_GCAL_LSB, LSB << 8,
                                  REGMASK_CHX_GCAL0_LSB));
    case 3:
      return (writeRegisterMasked(REG_CH3_GCAL_MSB, MSB, 0xFFFF) &&
              writeRegisterMasked(REG_CH3_GCAL_LSB, LSB << 8,
                                  REGMASK_CHX_GCAL0_LSB));
  };
  return false;
}

// Check hardware data ready pin
bool ADS131M04::isDataReady(void) { return digitalRead(DRDY_PIN) == LOW; }

// Display all register values
void ADS131M04::dumpRegs(Print* printer) {
  char out[7] = {'\0'};

  sprintf(out, "0x%.4x", readRegister(REG_ID));
  printer->print("ID: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_STATUS));
  printer->print("STATUS: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_MODE));
  printer->print("MODE: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CLOCK));
  printer->print("CLOCK: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_GAIN));
  printer->print("GAIN: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CFG));
  printer->print("CFG: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_THRSHLD_MSB));
  printer->print("THRSHLD_MSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_THRSHLD_LSB));
  printer->print("THRSHLD_LSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH0_CFG));
  printer->print("CH0_CFG: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH0_OCAL_MSB));
  printer->print("CH0_OCAL_MSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH0_OCAL_LSB));
  printer->print("CH0_OCAL_LSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH0_GCAL_MSB));
  printer->print("CH0_GCAL_MSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH0_GCAL_LSB));
  printer->print("CH0_GCAL_LSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH1_CFG));
  printer->print("CH1_CFG: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH1_OCAL_MSB));
  printer->print("CH1_OCAL_MSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH1_OCAL_LSB));
  printer->print("CH1_OCAL_LSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH1_GCAL_MSB));
  printer->print("CH1_GCAL_MSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH1_GCAL_LSB));
  printer->print("CH1_GCAL_LSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH2_CFG));
  printer->print("CH2_CFG: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH2_OCAL_MSB));
  printer->print("CH2_OCAL_MSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH2_OCAL_LSB));
  printer->print("CH2_OCAL_LSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH2_GCAL_MSB));
  printer->print("CH2_GCAL_MSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH2_GCAL_LSB));
  printer->print("CH2_GCAL_LSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH3_CFG));
  printer->print("CH3_CFG: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH3_OCAL_MSB));
  printer->print("CH3_OCAL_MSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH3_OCAL_LSB));
  printer->print("CH3_OCAL_LSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH3_GCAL_MSB));
  printer->print("CH3_GCAL_MSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_CH3_GCAL_LSB));
  printer->print("CH3_GCAL_LSB: ");
  printer->println(out);

  sprintf(out, "0x%.4x", readRegister(REG_MAP_CRC));
  printer->print("REGMAP_CRC: ");
  printer->println(out);
}
