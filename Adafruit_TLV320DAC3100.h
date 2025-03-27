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

#include "Adafruit_TLV320DAC3100_typedefs.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
#include <Wire.h>

#define TLV320DAC3100_I2CADDR_DEFAULT 0x18 ///< Default I2C address

#define TLV320DAC3100_REG_PAGE_SELECT 0x00 ///< Page select register
#define TLV320DAC3100_REG_RESET 0x01       ///< Reset register
#define TLV320DAC3100_REG_OT_FLAG 0x03     ///< Over-temperature flag register
#define TLV320DAC3100_REG_CLOCK_MUX1 0x04  ///< Clock muxing control register 1
#define TLV320DAC3100_REG_PLL_PROG_PR 0x05 ///< PLL P and R values
#define TLV320DAC3100_REG_PLL_PROG_J 0x06  ///< PLL J value
#define TLV320DAC3100_REG_PLL_PROG_D_MSB 0x07   ///< PLL D value MSB
#define TLV320DAC3100_REG_PLL_PROG_D_LSB 0x08   ///< PLL D value LSB
#define TLV320DAC3100_REG_NDAC 0x0B             ///< NDAC divider value
#define TLV320DAC3100_REG_MDAC 0x0C             ///< MDAC divider value
#define TLV320DAC3100_REG_DOSR 0x0D             ///< DOSR divider value MSB/LSB
#define TLV320DAC3100_REG_DOSR_MSB 0x0D         ///< DOSR divider value MSB
#define TLV320DAC3100_REG_DOSR_LSB 0x0E         ///< DOSR divider value LSB
#define TLV320DAC3100_REG_CLKOUT_MUX 0x19       ///< CLKOUT MUX register
#define TLV320DAC3100_REG_CLKOUT_M 0x1A         ///< CLKOUT M divider value
#define TLV320DAC3100_REG_CODEC_IF_CTRL1 0x1B   ///< Codec Interface Control 1
#define TLV320DAC3100_REG_DATA_SLOT_OFFSET 0x1C ///< Data-slot offset register
#define TLV320DAC3100_REG_BCLK_N 0x1E           ///< BCLK N divider value
#define TLV320DAC3100_REG_DAC_FLAG 0x25         ///< DAC Flag register
#define TLV320DAC3100_REG_DAC_FLAG2 0x26        ///< DAC Flag register 2
#define TLV320DAC3100_REG_INT1_CTRL 0x30        ///< INT1 Control Register
#define TLV320DAC3100_REG_INT2_CTRL 0x31        ///< INT2 Control Register
#define TLV320DAC3100_REG_GPIO1_CTRL 0x33 ///< GPIO1 In/Out Pin Control Register
#define TLV320DAC3100_REG_DIN_CTRL 0x36   ///< DIN Pin Control Register
#define TLV320DAC3100_REG_DAC_PRB                                              \
  0x3C ///< DAC Processing Block Selection Register
#define TLV320DAC3100_REG_DAC_DATAPATH 0x3F ///< DAC Data-Path Setup Register
#define TLV320DAC3100_REG_DAC_VOL_CTRL 0x40 ///< DAC Volume Control Register
#define TLV320DAC3100_REG_DAC_LVOL 0x41 ///< DAC Left Volume Control Register
#define TLV320DAC3100_REG_DAC_RVOL 0x42 ///< DAC Right Volume Control Register
#define TLV320DAC3100_REG_HEADSET_DETECT 0x43 ///< Headset Detection Register
#define TLV320DAC3100_REG_BEEP_L 0x47         ///< Left Beep Generator Register
#define TLV320DAC3100_REG_BEEP_R 0x48         ///< Right Beep Generator Register
#define TLV320DAC3100_REG_BEEP_LEN_MSB 0x49   ///< Beep Length MSB Register
#define TLV320DAC3100_REG_BEEP_LEN_MID                                         \
  0x4A ///< Beep Length Middle Bits Register
#define TLV320DAC3100_REG_BEEP_LEN_LSB 0x4B ///< Beep Length LSB Register
#define TLV320DAC3100_REG_BEEP_SIN_MSB 0x4C ///< Beep Sin(x) MSB Register
#define TLV320DAC3100_REG_BEEP_SIN_LSB 0x4D ///< Beep Sin(x) LSB Register
#define TLV320DAC3100_REG_BEEP_COS_MSB 0x4E ///< Beep Cos(x) MSB Register
#define TLV320DAC3100_REG_BEEP_COS_LSB 0x4F ///< Beep Cos(x) LSB Register
#define TLV320DAC3100_REG_VOL_ADC_CTRL                                         \
  0x74 ///< VOL/MICDET-Pin SAR ADC Control Register
#define TLV320DAC3100_REG_VOL_ADC_READ 0x75 ///< VOL/MICDET-Pin Gain Register

// Page 1
#define TLV320DAC3100_REG_BCLK_CTRL2 0x1D ///< BCLK Control Register 2
#define TLV320DAC3100_REG_HP_SPK_ERR_CTL                                       \
  0x1E ///< Headphone and Speaker Error Control Register
#define TLV320DAC3100_REG_HP_DRIVERS 0x1F ///< Headphone Drivers Register
#define TLV320DAC3100_REG_SPK_AMP 0x20 ///< Class-D Speaker Amplifier Register
#define TLV320DAC3100_REG_HP_POP                                               \
  0x21 ///< HP Output Drivers POP Removal Settings Register
#define TLV320DAC3100_REG_PGA_RAMP                                             \
  0x22 ///< Output Driver PGA Ramp-Down Period Control Register
#define TLV320DAC3100_REG_OUT_ROUTING                                          \
  0x23                                 ///< DAC Output Mixer Routing Register
#define TLV320DAC3100_REG_HPL_VOL 0x24 ///< Left Analog Volume to HPL Register
#define TLV320DAC3100_REG_HPR_VOL 0x25 ///< Right Analog Volume to HPR Register
#define TLV320DAC3100_REG_SPK_VOL 0x26 ///< Left Analog Volume to SPK Register
#define TLV320DAC3100_REG_HPL_DRIVER 0x28 ///< HPL Driver Register
#define TLV320DAC3100_REG_HPR_DRIVER 0x29 ///< HPR Driver Register
#define TLV320DAC3100_REG_SPK_DRIVER 0x2A ///< Class-D Speaker Driver Register
#define TLV320DAC3100_REG_HP_DRIVER_CTRL 0x2C ///< HP Driver Control Register
#define TLV320DAC3100_REG_MICBIAS 0x2E  ///< MICBIAS Configuration Register
#define TLV320DAC3100_REG_INPUT_CM 0x32 ///< Input Common Mode Settings Register
#define TLV320DAC3100_REG_TIMER_MCLK_DIV                                       \
  0x10 ///< Timer Clock MCLK Divider Register
#define TLV320DAC3100_REG_IRQ_FLAGS_STICKY                                     \
  0x2C                                   ///< Interrupt Flags - Sticky Register
#define TLV320DAC3100_REG_IRQ_FLAGS 0x2E ///< Interrupt Flags - DAC Register

// IRQ Flag bits
#define TLV320DAC3100_IRQ_HPL_SHORT                                            \
  0x80 ///< Short circuit detected at HPL / left class-D driver
#define TLV320DAC3100_IRQ_HPR_SHORT                                            \
  0x40 ///< Short circuit detected at HPR / right class-D driver
#define TLV320DAC3100_IRQ_BUTTON_PRESS 0x20 ///< Headset button pressed
#define TLV320DAC3100_IRQ_HEADSET_DETECT                                       \
  0x10 ///< Headset insertion detected (1) or removal detected (0)
#define TLV320DAC3100_IRQ_LEFT_DRC                                             \
  0x08 ///< Left DAC signal power greater than DRC threshold
#define TLV320DAC3100_IRQ_RIGHT_DRC                                            \
  0x04 ///< Right DAC signal power greater than DRC threshold

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
  bool getDACFlags(bool *left_dac_powered, bool *hpl_powered,
                   bool *left_classd_powered, bool *right_dac_powered,
                   bool *hpr_powered, bool *right_classd_powered,
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
  bool setBCLKConfig(bool invert_bclk, bool active_when_powered_down,
                     tlv320dac3100_bclk_src_t source);
  bool getBCLKConfig(bool *invert_bclk, bool *active_when_powered_down,
                     tlv320dac3100_bclk_src_t *source);

  bool validatePLLConfig(uint8_t P, uint8_t R, uint8_t J, uint16_t D,
                         float pll_clkin);

  bool setCodecInterface(tlv320dac3100_format_t format,
                         tlv320dac3100_data_len_t len, bool bclk_out = false,
                         bool wclk_out = false);
  bool getCodecInterface(tlv320dac3100_format_t *format,
                         tlv320dac3100_data_len_t *len, bool *bclk_out,
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

  bool
  setDACDataPath(bool left_dac_on, bool right_dac_on,
                 tlv320_dac_path_t left_path = TLV320_DAC_PATH_NORMAL,
                 tlv320_dac_path_t right_path = TLV320_DAC_PATH_NORMAL,
                 tlv320_volume_step_t volume_step = TLV320_VOLUME_STEP_1SAMPLE);
  bool getDACDataPath(bool *left_dac_on, bool *right_dac_on,
                      tlv320_dac_path_t *left_path,
                      tlv320_dac_path_t *right_path,
                      tlv320_volume_step_t *volume_step);

  bool
  setDACVolumeControl(bool left_mute, bool right_mute,
                      tlv320_vol_control_t control = TLV320_VOL_INDEPENDENT);
  bool getDACVolumeControl(bool *left_mute, bool *right_mute,
                           tlv320_vol_control_t *control);
  bool setChannelVolume(bool right_channel, float dB);
  float getChannelVolume(bool right_channel);

  bool setHeadsetDetect(
      bool enable,
      tlv320_detect_debounce_t detect_debounce = TLV320_DEBOUNCE_16MS,
      tlv320_button_debounce_t button_debounce = TLV320_BTN_DEBOUNCE_0MS);
  tlv320_headset_status_t getHeadsetStatus(void);

  bool isBeeping(void);
  bool enableBeep(bool enable);
  bool setBeepVolume(int8_t left_dB,
                     int8_t right_dB = -100); // -100 is sentinel value
  bool setBeepLength(uint32_t samples);
  bool setBeepSinCos(uint16_t sin_val, uint16_t cos_val);
  bool configureBeepTone(float frequency, uint32_t duration_ms,
                         uint32_t sample_rate);

  bool configVolADC(bool pin_control, bool use_mclk,
                    tlv320_vol_hyst_t hysteresis, tlv320_vol_rate_t rate);
  float readVolADCdB(void);

  bool resetSpeakerOnSCD(bool reset);
  bool resetHeadphoneOnSCD(bool reset);
  bool
  configureHeadphoneDriver(bool left_powered, bool right_powered,
                           tlv320_hp_common_t common = TLV320_HP_COMMON_1_35V,
                           bool powerDownOnSCD = false);
  /*!
   * @brief Check if headphone outputs have detected a short circuit
   * @return true if short circuit detected, false if not
   */
  bool isHeadphoneShorted(void);
  bool enableSpeaker(bool en);
  bool speakerEnabled(void);
  bool isSpeakerShorted(void);

  bool
  configureHeadphonePop(bool wait_for_powerdown = true,
                        tlv320_hp_time_t powerup_time = TLV320_HP_TIME_304MS,
                        tlv320_ramp_time_t ramp_time = TLV320_RAMP_4MS);
  bool setSpeakerWaitTime(tlv320_spk_wait_t wait_time = TLV320_SPK_WAIT_0MS);
  bool
  configureAnalogInputs(tlv320_dac_route_t left_dac = TLV320_DAC_ROUTE_NONE,
                        tlv320_dac_route_t right_dac = TLV320_DAC_ROUTE_NONE,
                        bool left_ain1 = false, bool left_ain2 = false,
                        bool right_ain2 = false,
                        bool hpl_routed_to_hpr = false);

  bool setHPLVolume(bool route_enabled, uint8_t gain = 0x7F);
  bool setHPRVolume(bool route_enabled, uint8_t gain = 0x7F);
  bool setSPKVolume(bool route_enabled, uint8_t gain = 0x7F);
  bool configureHPL_PGA(uint8_t gain_db = 0, bool unmute = true);
  bool configureHPR_PGA(uint8_t gain_db = 0, bool unmute = true);
  bool configureSPK_PGA(tlv320_spk_gain_t gain = TLV320_SPK_GAIN_6DB,
                        bool unmute = true);
  bool isHPLGainApplied(void);
  bool isHPRGainApplied(void);
  bool isSPKGainApplied(void);
  bool headphoneLineout(bool left, bool right);
  bool configMicBias(bool power_down = false, bool always_on = false,
                     tlv320_micbias_volt_t voltage = TLV320_MICBIAS_OFF);
  bool setInputCommonMode(bool ain1_cm, bool ain2_cm);

  bool configDelayDivider(bool use_mclk = true, uint8_t divider = 1);

  bool configurePLL(uint32_t mclk_freq, uint32_t desired_freq,
                    float max_error = 0.001);

  uint8_t readIRQflags(bool sticky = false);

  uint8_t readRegister(uint8_t page, uint8_t reg);

private:
  bool setPage(uint8_t page);
  uint8_t getPage(void);

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
};

#endif
