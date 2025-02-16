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
#define TLV320DAC3100_REG_INT1_CTRL   0x30  ///< INT1 Control Register
#define TLV320DAC3100_REG_INT2_CTRL   0x31  ///< INT2 Control Register
#define TLV320DAC3100_REG_GPIO1_CTRL  0x33  ///< GPIO1 In/Out Pin Control Register
#define TLV320DAC3100_REG_DIN_CTRL    0x36  ///< DIN Pin Control Register
#define TLV320DAC3100_REG_DAC_PRB    0x3C  ///< DAC Processing Block Selection Register
#define TLV320DAC3100_REG_DAC_DATAPATH  0x3F  ///< DAC Data-Path Setup Register
#define TLV320DAC3100_REG_DAC_VOL_CTRL  0x40  ///< DAC Volume Control Register
#define TLV320DAC3100_REG_DAC_LVOL  0x41  ///< DAC Left Volume Control Register
#define TLV320DAC3100_REG_DAC_RVOL  0x42  ///< DAC Right Volume Control Register
#define TLV320DAC3100_REG_HEADSET_DETECT  0x43  ///< Headset Detection Register
#define TLV320DAC3100_REG_BEEP_L  0x47  ///< Left Beep Generator Register
#define TLV320DAC3100_REG_BEEP_R  0x48  ///< Right Beep Generator Register
#define TLV320DAC3100_REG_BEEP_LEN_MSB    0x49  ///< Beep Length MSB Register
#define TLV320DAC3100_REG_BEEP_LEN_MID    0x4A  ///< Beep Length Middle Bits Register
#define TLV320DAC3100_REG_BEEP_LEN_LSB    0x4B  ///< Beep Length LSB Register
#define TLV320DAC3100_REG_BEEP_SIN_MSB    0x4C  ///< Beep Sin(x) MSB Register
#define TLV320DAC3100_REG_BEEP_SIN_LSB    0x4D  ///< Beep Sin(x) LSB Register
#define TLV320DAC3100_REG_BEEP_COS_MSB    0x4E  ///< Beep Cos(x) MSB Register
#define TLV320DAC3100_REG_BEEP_COS_LSB    0x4F  ///< Beep Cos(x) LSB Register
#define TLV320DAC3100_REG_VOL_ADC_CTRL    0x74  ///< VOL/MICDET-Pin SAR ADC Control Register
#define TLV320DAC3100_REG_VOL_ADC_READ    0x75  ///< VOL/MICDET-Pin Gain Register

// Page 1
#define TLV320DAC3100_REG_HP_SPK_ERR_CTL  0x1E  ///< Headphone and Speaker Error Control Register
#define TLV320DAC3100_REG_HP_DRIVERS  0x1F  ///< Headphone Drivers Register

/*!
 * @brief Headset detection debounce time options
 */
typedef enum {
  TLV320_DEBOUNCE_16MS = 0b000,    ///< 16ms debounce (2ms clock)
  TLV320_DEBOUNCE_32MS = 0b001,    ///< 32ms debounce (4ms clock)
  TLV320_DEBOUNCE_64MS = 0b010,    ///< 64ms debounce (8ms clock)
  TLV320_DEBOUNCE_128MS = 0b011,   ///< 128ms debounce (16ms clock)
  TLV320_DEBOUNCE_256MS = 0b100,   ///< 256ms debounce (32ms clock)
  TLV320_DEBOUNCE_512MS = 0b101,   ///< 512ms debounce (64ms clock)
} tlv320_detect_debounce_t;

/*!
 * @brief Button press debounce time options
 */
typedef enum {
  TLV320_BTN_DEBOUNCE_0MS = 0b00,   ///< No debounce
  TLV320_BTN_DEBOUNCE_8MS = 0b01,   ///< 8ms debounce (1ms clock)
  TLV320_BTN_DEBOUNCE_16MS = 0b10,  ///< 16ms debounce (2ms clock)
  TLV320_BTN_DEBOUNCE_32MS = 0b11,  ///< 32ms debounce (4ms clock)
} tlv320_button_debounce_t;

/*!
 * @brief Headset detection status
 */
typedef enum {
  TLV320_HEADSET_NONE = 0b00,        ///< No headset detected
  TLV320_HEADSET_WITHOUT_MIC = 0b01, ///< Headset without microphone
  TLV320_HEADSET_WITH_MIC = 0b11,    ///< Headset with microphone
} tlv320_headset_status_t;

/*!
 * @brief DAC channel data path options
 */
typedef enum {
  TLV320_DAC_PATH_OFF = 0b00,      ///< DAC data path off
  TLV320_DAC_PATH_NORMAL = 0b01,    ///< Normal path (L->L or R->R)
  TLV320_DAC_PATH_SWAPPED = 0b10,   ///< Swapped path (R->L or L->R)
  TLV320_DAC_PATH_MIXED = 0b11,     ///< Mixed L+R path
} tlv320_dac_path_t;

/*!
 * @brief DAC volume control soft stepping options
 */
typedef enum {
  TLV320_VOLUME_STEP_1SAMPLE = 0b00,    ///< One step per sample
  TLV320_VOLUME_STEP_2SAMPLE = 0b01,    ///< One step per two samples
  TLV320_VOLUME_STEP_DISABLED = 0b10,   ///< Soft stepping disabled
} tlv320_volume_step_t;

/*!
 * @brief DAC volume control configuration options
 */
typedef enum {
  TLV320_VOL_INDEPENDENT = 0b00,    ///< Left and right channels independent
  TLV320_VOL_LEFT_TO_RIGHT = 0b01,  ///< Left follows right volume
  TLV320_VOL_RIGHT_TO_LEFT = 0b10,  ///< Right follows left volume
} tlv320_vol_control_t;


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
 * @brief GPIO1 pin mode options
 */
typedef enum {
  TLV320_GPIO1_DISABLED = 0b0000,    ///< GPIO1 disabled (input and output buffers powered down)
  TLV320_GPIO1_INPUT_MODE = 0b0001,  ///< Input mode (secondary BCLK/WCLK/DIN input or ClockGen)
  TLV320_GPIO1_GPI = 0b0010,         ///< General-purpose input
  TLV320_GPIO1_GPO = 0b0011,         ///< General-purpose output
  TLV320_GPIO1_CLKOUT = 0b0100,      ///< CLKOUT output
  TLV320_GPIO1_INT1 = 0b0101,        ///< INT1 output
  TLV320_GPIO1_INT2 = 0b0110,        ///< INT2 output
  TLV320_GPIO1_BCLK_OUT = 0b1000,    ///< Secondary BCLK output for codec interface
  TLV320_GPIO1_WCLK_OUT = 0b1001,    ///< Secondary WCLK output for codec interface
} tlv320_gpio1_mode_t;


/*!
 * @brief DIN pin mode options
 */
typedef enum {
  TLV320_DIN_DISABLED = 0b00,    ///< DIN disabled (input buffer powered down)
  TLV320_DIN_ENABLED = 0b01,     ///< DIN enabled (for codec interface/ClockGen)
  TLV320_DIN_GPI = 0b10,         ///< DIN used as general-purpose input
} tlv320_din_mode_t;

/*!
 * @brief Volume ADC hysteresis options
 */
typedef enum {
  TLV320_VOL_HYST_NONE = 0b00,   ///< No hysteresis
  TLV320_VOL_HYST_1BIT = 0b01,   ///< ±1 bit hysteresis
  TLV320_VOL_HYST_2BIT = 0b10,   ///< ±2 bit hysteresis
} tlv320_vol_hyst_t;

/*!
 * @brief Volume ADC throughput rates
 */
typedef enum {
  TLV320_VOL_RATE_15_625HZ = 0b000,  ///< 15.625 Hz (MCLK) or 10.68 Hz (RC)
  TLV320_VOL_RATE_31_25HZ = 0b001,   ///< 31.25 Hz (MCLK) or 21.35 Hz (RC)
  TLV320_VOL_RATE_62_5HZ = 0b010,    ///< 62.5 Hz (MCLK) or 42.71 Hz (RC)
  TLV320_VOL_RATE_125HZ = 0b011,     ///< 125 Hz (MCLK) or 85.2 Hz (RC)
  TLV320_VOL_RATE_250HZ = 0b100,     ///< 250 Hz (MCLK) or 170 Hz (RC)
  TLV320_VOL_RATE_500HZ = 0b101,     ///< 500 Hz (MCLK) or 340 Hz (RC)
  TLV320_VOL_RATE_1KHZ = 0b110,      ///< 1 kHz (MCLK) or 680 Hz (RC)
  TLV320_VOL_RATE_2KHZ = 0b111,      ///< 2 kHz (MCLK) or 1.37 kHz (RC)
} tlv320_vol_rate_t;


typedef enum {
  TLV320_HP_COMMON_1_35V = 0b00,  ///< Common-mode voltage 1.35V
  TLV320_HP_COMMON_1_50V = 0b01,  ///< Common-mode voltage 1.50V
  TLV320_HP_COMMON_1_65V = 0b10,  ///< Common-mode voltage 1.65V
  TLV320_HP_COMMON_1_80V = 0b11,  ///< Common-mode voltage 1.80V
} tlv320_hp_common_t;
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
                    bool agc_noise, bool over_current, bool multiple_pulse);
  bool setInt2Source(bool headset_detect, bool button_press, bool dac_drc, 
                    bool agc_noise, bool over_current, bool multiple_pulse);

  bool setGPIO1Mode(tlv320_gpio1_mode_t mode);
  tlv320_gpio1_mode_t getGPIO1Mode(void);
  bool setGPIO1Output(bool value);
  bool getGPIO1Input(void);

  bool setDINMode(tlv320_din_mode_t mode);
  tlv320_din_mode_t getDINMode(void);
  bool getDINInput(void);

  bool setDACProcessingBlock(uint8_t block_number);
  uint8_t getDACProcessingBlock(void);

  bool setDACDataPath(bool left_dac_on, bool right_dac_on,
                     tlv320_dac_path_t left_path = TLV320_DAC_PATH_NORMAL,
                     tlv320_dac_path_t right_path = TLV320_DAC_PATH_NORMAL,
                     tlv320_volume_step_t volume_step = TLV320_VOLUME_STEP_1SAMPLE);
  bool getDACDataPath(bool *left_dac_on, bool *right_dac_on,
                     tlv320_dac_path_t *left_path, tlv320_dac_path_t *right_path,
                     tlv320_volume_step_t *volume_step);

  bool setDACVolumeControl(bool left_mute, bool right_mute, 
                          tlv320_vol_control_t control = TLV320_VOL_INDEPENDENT);
  bool getDACVolumeControl(bool *left_mute, bool *right_mute,
                          tlv320_vol_control_t *control);
  bool setChannelVolume(bool right_channel, float dB);
  float getChannelVolume(bool right_channel);

  bool setHeadsetDetect(bool enable, 
                       tlv320_detect_debounce_t detect_debounce = TLV320_DEBOUNCE_16MS,
                       tlv320_button_debounce_t button_debounce = TLV320_BTN_DEBOUNCE_0MS);
  tlv320_headset_status_t getHeadsetStatus(void);

  bool setBeepVolume(int8_t left_dB, int8_t right_dB = -100);  // -100 is sentinel value
  bool setBeepLength(uint32_t samples);
  bool setBeepSinCos(uint16_t sin_val, uint16_t cos_val);
  bool configVolADC(bool pin_control, bool use_mclk, 
                         tlv320_vol_hyst_t hysteresis,
                         tlv320_vol_rate_t rate);
  float readVolADCdB(void);


  bool resetSpeakerOnSCD(bool reset);
  bool resetHeadphoneOnSCD(bool reset);
  bool configureHeadphoneDriver(bool left_powered, bool right_powered,
                               tlv320_hp_common_t common = TLV320_HP_COMMON_1_35V,
                               bool powerDownOnSCD = false);
  bool isHeadphoneShorted(void);

private:
  bool setPage(uint8_t page);
  uint8_t getPage(void);

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
};

#endif
