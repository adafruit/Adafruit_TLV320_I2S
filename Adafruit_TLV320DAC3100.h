/*!
 * @file Adafruit_TLV320DAC3100.h
 *
 * Arduino library for the TI TLV320DAC3100 stereo DAC with headphone amplifier
 * 
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 */

#ifndef _ADAFRUIT_TLV320DAC3100_H
#define _ADAFRUIT_TLV320DAC3100_H

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
#include <Wire.h>

#define TLV320DAC3100_I2CADDR_DEFAULT 0x18 ///< Default I2C address

#define TLV320DAC3100_REG_PAGE_SELECT    0x00    ///< Page select register
#define TLV320DAC3100_REG_RESET          0x01    ///< Reset register
#define TLV320DAC3100_REG_OT_FLAG        0x03    ///< Over-temperature flag register
#define TLV320DAC3100_REG_CLOCK_MUX1     0x04    ///< Clock muxing control register 1
#define TLV320DAC3100_REG_PLL_PROG_PR    0x05    ///< PLL P and R values
#define TLV320DAC3100_REG_PLL_PROG_J     0x06    ///< PLL J value
#define TLV320DAC3100_REG_PLL_PROG_D_MSB 0x07    ///< PLL D value MSB
#define TLV320DAC3100_REG_PLL_PROG_D_LSB 0x08    ///< PLL D value LSB
#define TLV320DAC3100_REG_NDAC          0x0B    ///< NDAC divider value
#define TLV320DAC3100_REG_MDAC          0x0C    ///< MDAC divider value
#define TLV320DAC3100_REG_DOSR          0x0D    ///< DOSR divider value MSB/LSB
#define TLV320DAC3100_REG_DOSR_MSB      0x0D    ///< DOSR divider value MSB
#define TLV320DAC3100_REG_DOSR_LSB      0x0E    ///< DOSR divider value LSB
#define TLV320DAC3100_REG_CLKOUT_MUX    0x19    ///< CLKOUT MUX register
#define TLV320DAC3100_REG_CLKOUT_M      0x1A    ///< CLKOUT M divider value
#define TLV320DAC3100_REG_CODEC_IF_CTRL1    0x1B    ///< Codec Interface Control 1
#define TLV320DAC3100_REG_DATA_SLOT_OFFSET    0x1C    ///< Data-slot offset register
#define TLV320DAC3100_REG_BCLK_N        0x1E    ///< BCLK N divider value
#define TLV320DAC3100_REG_DAC_FLAG      0x25    ///< DAC Flag register
#define TLV320DAC3100_REG_DAC_FLAG2     0x26    ///< DAC Flag register 2
#define TLV320DAC3100_REG_INT1_CTRL     0x30    ///< INT1 Control Register
#define TLV320DAC3100_REG_INT2_CTRL     0x31    ///< INT2 Control Register

/*!
 * @brief Clock source options for CODEC_CLKIN
 */
typedef enum {
  TLV320DAC3100_CODEC_CLKIN_MCLK = 0b00,    ///< MCLK pin is the source
  TLV320DAC3100_CODEC_CLKIN_BCLK = 0b01,    ///< BCLK pin is the source
  TLV320DAC3100_CODEC_CLKIN_GPIO1 = 0b10,   ///< GPIO1 pin is the source
  TLV320DAC3100_CODEC_CLKIN_PLL = 0b11,     ///< PLL_CLK pin is the source
} tlv320dac3100_codec_clkin_t;

/*!
 * @brief Clock source options for PLL_CLKIN
 */
typedef enum {
  TLV320DAC3100_PLL_CLKIN_MCLK = 0b00,     ///< MCLK pin is the source
  TLV320DAC3100_PLL_CLKIN_BCLK = 0b01,     ///< BCLK pin is the source
  TLV320DAC3100_PLL_CLKIN_GPIO1 = 0b10,    ///< GPIO1 pin is the source
  TLV320DAC3100_PLL_CLKIN_DIN = 0b11       ///< DIN pin is the source
} tlv320dac3100_pll_clkin_t;


/*!
 * @brief Clock divider input source options
 */
typedef enum {
  TLV320DAC3100_CDIV_CLKIN_MCLK = 0b000,         ///< MCLK (device pin)
  TLV320DAC3100_CDIV_CLKIN_BCLK = 0b001,         ///< BCLK (device pin)
  TLV320DAC3100_CDIV_CLKIN_DIN = 0b010,          ///< DIN (for systems where DAC is not required)
  TLV320DAC3100_CDIV_CLKIN_PLL = 0b011,          ///< PLL_CLK (generated on-chip)
  TLV320DAC3100_CDIV_CLKIN_DAC = 0b100,          ///< DAC_CLK (DAC DSP clock - generated on-chip)
  TLV320DAC3100_CDIV_CLKIN_DAC_MOD = 0b101,      ///< DAC_MOD_CLK (generated on-chip)
} tlv320dac3100_cdiv_clkin_t;


/*!
 * @brief Data length for I2S interface
 */
typedef enum {
  TLV320DAC3100_DATA_LEN_16 = 0b00,      ///< 16 bits
  TLV320DAC3100_DATA_LEN_20 = 0b01,      ///< 20 bits
  TLV320DAC3100_DATA_LEN_24 = 0b10,      ///< 24 bits
  TLV320DAC3100_DATA_LEN_32 = 0b11,      ///< 32 bits
} tlv320dac3100_data_len_t;

/*!
 * @brief Data format for I2S interface
 */
typedef enum {
  TLV320DAC3100_FORMAT_I2S = 0b00,       ///< I2S format
  TLV320DAC3100_FORMAT_DSP = 0b01,       ///< DSP format
  TLV320DAC3100_FORMAT_RJF = 0b10,       ///< Right justified format
  TLV320DAC3100_FORMAT_LJF = 0b11,       ///< Left justified format
} tlv320dac3100_format_t;

/*!
 * @brief Class to interact with TLV320DAC3100 DAC
 */
class Adafruit_TLV320DAC3100 {
public:
  Adafruit_TLV320DAC3100();
  bool begin(uint8_t i2c_addr = TLV320DAC3100_I2CADDR_DEFAULT,
            TwoWire *wire = &Wire);

  bool reset(void);
  bool isOvertemperature(void);


  bool setPLLClockInput(tlv320dac3100_pll_clkin_t clkin);
  tlv320dac3100_pll_clkin_t getPLLClockInput(void);
  bool setCodecClockInput(tlv320dac3100_codec_clkin_t clkin);
  tlv320dac3100_codec_clkin_t getCodecClockInput(void);
  bool setClockDividerInput(tlv320dac3100_cdiv_clkin_t clkin);
  tlv320dac3100_cdiv_clkin_t getClockDividerInput(void);


  bool powerPLL(bool on);
  bool isPLLpowered(void);
  bool setPLLValues(uint8_t P, uint8_t R, uint8_t J, uint16_t D);
  bool getPLLValues(uint8_t *P, uint8_t *R, uint8_t *J, uint16_t *D);
  bool getDACFlags(bool *left_dac_powered, bool *hpl_powered, bool *left_classd_powered,
                  bool *right_dac_powered, bool *hpr_powered, bool *right_classd_powered,
                  bool *left_pga_gain_ok, bool *right_pga_gain_ok);

  bool setNDAC(bool enable, uint8_t val);
  bool getNDAC(bool *enabled, uint8_t *val);
  bool setMDAC(bool enable, uint8_t val);
  bool getMDAC(bool *enabled, uint8_t *val);
  bool setDOSR(uint16_t val);
  bool getDOSR(uint16_t *val);
  bool setCLKOUT_M(bool enable, uint8_t val);
  bool getCLKOUT_M(bool *enabled, uint8_t *val);
  bool setBCLKoffset(uint8_t offset);
  bool getBCLKoffset(uint8_t *offset);
  bool setBCLK_N(bool enable, uint8_t val);
  bool getBCLK_N(bool *enabled, uint8_t *val);

  bool setCodecInterface(tlv320dac3100_format_t format, 
                        tlv320dac3100_data_len_t len,
                        bool bclk_out = false,
                        bool wclk_out = false);
  bool getCodecInterface(tlv320dac3100_format_t *format,
                        tlv320dac3100_data_len_t *len,
                        bool *bclk_out,
                        bool *wclk_out);

  bool setInt1Source(bool headset_detect, bool button_press, bool dac_drc, 
                    bool agc_noise, bool over_current, bool pwr_status);
  bool setInt2Source(bool headset_detect, bool button_press, bool dac_drc, 
                    bool agc_noise, bool over_current, bool pwr_status);

private:
  bool setPage(uint8_t page);
  uint8_t getPage(void);

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
};

#endif
