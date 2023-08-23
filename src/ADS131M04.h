#ifndef ADS131M04_h
#define ADS131M04_h

#include "Arduino.h"
#include "Print.h"
#include "SPI.h"
#include "crc16.h"

// ================ Command Macros ================

// Commands
#define CMD_NULL 0x0000
#define CMD_RESET 0x0011
#define CMD_STANDBY 0x0022
#define CMD_WAKEUP 0x0033
#define CMD_LOCK 0x0555
#define CMD_UNLOCK 0x0655
#define CMD_READ_REG 0xA000
#define CMD_WRITE_REG 0x6000

// Command Responses
#define RSP_RESET_OK 0xFF24
#define RSP_STANDBY_OK 0x0022
#define RSP_WAKEUP_OK 0x0033
#define RSP_LOCK_OK 0x0555
#define RSP_UNLOCK_OK 0x0655

// ================ Settings Macros ================

// Data Ready Pin Idle State
#define DRDY_STATE_LOGIC_HIGH 0  // Default
#define DRDY_STATE_HI_Z 1

// Power Modes
#define POWER_MODE_VERY_LOW_POWER 0
#define POWER_MODE_LOW_POWER 1
#define POWER_MODE_HIGH_RESOLUTION 2  // Default

// Input MUX
#define INPUT_CHANNEL_MUX_AIN0P_AIN0N 0  // Default
#define INPUT_CHANNEL_MUX_INPUT_SHORTED 1
#define INPUT_CHANNEL_MUX_POSITIVE_DC_TEST_SIGNAL 2
#define INPUT_CHANNEL_MUX_NEGATIVE_DC_TEST_SIGNAL 3

// Oversampling Ratio
#define OSR_64 8  // "Turbo mode"
#define OSR_128 0
#define OSR_256 1
#define OSR_512 2
#define OSR_1024 3  // Default
#define OSR_2018 4
#define OSR_4096 5
#define OSR_8192 6
#define OSR_16384 7

// PGA Gain
#define PGA_GAIN_1 0  // Default
#define PGA_GAIN_2 1
#define PGA_GAIN_4 2
#define PGA_GAIN_8 3
#define PGA_GAIN_16 4
#define PGA_GAIN_32 5
#define PGA_GAIN_64 6
#define PGA_GAIN_128 7

// Input Filter
#define FILTER_SINC 0
#define FILTER_FIR 2
#define FILTER_FIR_IIR 3

// Data Mode
#define WORD_LENGTH_16BITS 0
#define WORD_LENGTH_24BITS 1  // Default
#define WORD_LENGTH_32BITS_LSB_PAD 2
#define WORD_LENGTH_32BITS_SGN_EXT 3

// Global Chop Delay
#define CHOP_DELAY_2 0
#define CHOP_DELAY_4 1
#define CHOP_DELAY_8 2
#define CHOP_DELAY_16 3  // Default
#define CHOP_DELAY_32 4
#define CHOP_DELAY_64 5
#define CHOP_DELAY_128 6
#define CHOP_DELAY_256 7
#define CHOP_DELAY_512 8
#define CHOP_DELAY_1024 9
#define CHOP_DELAY_2048 10
#define CHOP_DELAY_4096 11
#define CHOP_DELAY_8192 12
#define CHOP_DELAY_16384 13
#define CHOP_DELAY_32768 14
#define CHOP_DELAY_65536 15

// ================ Register Addresses ================

// Info registers (read only)
#define REG_ID 0x00
#define REG_STATUS 0x01
#define REG_MAP_CRC 0x3E

// Global settings registers
#define REG_MODE 0x02
#define REG_CLOCK 0x03
#define REG_GAIN 0x04
#define REG_CFG 0x06
#define REG_THRSHLD_MSB 0x07
#define REG_THRSHLD_LSB 0x08

// Registers Channel 0 Specific
#define REG_CH0_CFG 0x09
#define REG_CH0_OCAL_MSB 0x0A
#define REG_CH0_OCAL_LSB 0x0B
#define REG_CH0_GCAL_MSB 0x0C
#define REG_CH0_GCAL_LSB 0x0D

// Registers Channel 1 Specific
#define REG_CH1_CFG 0x0E
#define REG_CH1_OCAL_MSB 0x0F
#define REG_CH1_OCAL_LSB 0x10
#define REG_CH1_GCAL_MSB 0x11
#define REG_CH1_GCAL_LSB 0x12

// Registers Channel 2 Specific
#define REG_CH2_CFG 0x13
#define REG_CH2_OCAL_MSB 0x14
#define REG_CH2_OCAL_LSB 0x15
#define REG_CH2_GCAL_MSB 0x16
#define REG_CH2_GCAL_LSB 0x17

// Registers Channel 3 Specific
#define REG_CH3_CFG 0x18
#define REG_CH3_OCAL_MSB 0x19
#define REG_CH3_OCAL_LSB 0x1A
#define REG_CH3_GCAL_MSB 0x1B
#define REG_CH3_GCAL_LSB 0x1C

// ================ Register Masks ================

// Mask READ_REG
#define REGMASK_CMD_READ_REG_ADDRESS 0x1F80
#define REGMASK_CMD_READ_REG_BYTES 0x007F

// Mask Register STATUS
#define REGMASK_STATUS_LOCK 0x8000
#define REGMASK_STATUS_RESYNC 0x4000
#define REGMASK_STATUS_REGMAP 0x2000
#define REGMASK_STATUS_CRC_ERR 0x1000
#define REGMASK_STATUS_CRC_TYPE 0x0800
#define REGMASK_STATUS_RESET 0x0400
#define REGMASK_STATUS_WLENGTH 0x0300
#define REGMASK_STATUS_DRDY3 0x0008
#define REGMASK_STATUS_DRDY2 0x0004
#define REGMASK_STATUS_DRDY1 0x0002
#define REGMASK_STATUS_DRDY0 0x0001

// Mask Register MODE
#define REGMASK_MODE_REG_CRC_EN 0x2000
#define REGMASK_MODE_RX_CRC_EN 0x1000
#define REGMASK_MODE_CRC_TYPE 0x0800
#define REGMASK_MODE_RESET 0x0400
#define REGMASK_MODE_WLENGTH 0x0300
#define REGMASK_MODE_TIMEOUT 0x0010
#define REGMASK_MODE_DRDY_SEL 0x000C
#define REGMASK_MODE_DRDY_HiZ 0x0002
#define REGMASK_MODE_DRDY_FMT 0x0001

// Mask Register CLOCK
#define REGMASK_CLOCK_CH3_EN 0x0800
#define REGMASK_CLOCK_CH2_EN 0x0400
#define REGMASK_CLOCK_CH1_EN 0x0200
#define REGMASK_CLOCK_CH0_EN 0x0100
#define REGMASK_CLOCK_OSR 0x003C
#define REGMASK_CLOCK_PWR 0x0003

// Mask Register GAIN
#define REGMASK_GAIN_PGAGAIN3 0x7000
#define REGMASK_GAIN_PGAGAIN2 0x0700
#define REGMASK_GAIN_PGAGAIN1 0x0070
#define REGMASK_GAIN_PGAGAIN0 0x0007

// Mask Register CFG
#define REGMASK_CFG_GC_DLY 0x1E00
#define REGMASK_CFG_GC_EN 0x0100
#define REGMASK_CFG_CD_ALLCH 0x0080
#define REGMASK_CFG_CD_NUM 0x0070
#define REGMASK_CFG_CD_LEN 0x000E
#define REGMASK_CFG_CD_EN 0x0001

// Mask Register THRSHLD_LSB
#define REGMASK_THRSHLD_LSB_CD_TH_LSB 0xFF00
#define REGMASK_THRSHLD_LSB_DCBLOCK 0x000F

// Mask Register CHX_CFG
#define REGMASK_CHX_CFG_PHASE 0xFFC0
#define REGMASK_CHX_CFG_DCBLKX_DIS0 0x0004
#define REGMASK_CHX_CFG_MUX 0x0003

// Mask Register CHX_OCAL_LSB
#define REGMASK_CHX_OCAL0_LSB 0xFF00

// Mask Register CHX_GCAL_LSB
#define REGMASK_CHX_GCAL0_LSB 0xFF00

// =============== Class declarations ===============

// Struct for ADC channel values
struct adcOutput {
  bool valid = false;
  int32_t raw_values[4] = {0};
  float voltages[4] = {0.0};
};

// ADC class
class ADS131M04 {
 public:
  // Pin constants
  uint8_t CS_PIN;
  uint8_t DRDY_PIN;

  // SPI Object
  SPIClass* spi;

  // CRC objects
  crc16 rx_crc;
  crc16 tx_crc;
  bool spi_crc_enabled = false;

  // ADC channel gains
  uint8_t pga_gains[4] = {1, 1, 1, 1};

  // Data word format
  uint8_t word_length = WORD_LENGTH_24BITS;
  uint8_t word_bytes = 3;

  bool begin(SPIClass* spi, SPISettings spi_settings, uint8_t cs_pin, uint8_t drdy_pin);
  adcOutput readADC(void);
  bool isDataReadySoft(uint16_t channel_mask);
  bool isDataReady(void);
  void dumpRegs(Print* printer = &Serial);

  // Command wrappers
  bool writeRegister(uint8_t address, uint16_t value);
  bool writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask);
  uint16_t readRegister(uint8_t address);
  bool resetSoft(void);
  bool enterStandby(void);
  bool wakeFromStandby(void);
  bool lockSPI(void);    // Not tested
  bool unlockSPI(void);  // Not tested

  // MODE register setter functions
  bool enableCRC(bool setting = 1);
  bool setWordLength(
      uint8_t word_length);  // Haven't tested word lengths other than 24
  bool setDrdySource(uint8_t drdySource);    // Not tested
  bool setDrdyIdleState(uint8_t drdyState);  // Not tested
  bool setDrdyFormat(uint8_t drdyFormat);    // Not tested

  // CLOCK register setter functions
  bool setChannelEnable(uint8_t channel, bool enable);  // Not tested
  bool setOSR(uint16_t osr);
  bool setPowerMode(uint8_t powerMode);  // Not tested

  // GAIN register setter function
  bool setChannelPGA(uint8_t channel, uint16_t pga);

  // CFG register setter functions
  bool setGlobalChopDelay(uint8_t delay);
  bool enableGlobalChop(bool setting = 1);

  // Individual channel parameters
  bool setInputChannelSelection(uint8_t channel, uint8_t input);
  bool setChannelOffsetCalibration(uint8_t channel, int32_t offset);
  bool setChannelGainCalibration(uint8_t channel, uint32_t gain);

 private:
  void resetSettings(void);
  uint32_t transfer_word(uint32_t data, int bytes = 2);
  uint16_t send_command(uint16_t cmd, int null_words = 5);
};

#endif
