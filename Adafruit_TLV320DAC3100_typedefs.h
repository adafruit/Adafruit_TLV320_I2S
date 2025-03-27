/*!
 * @file Adafruit_TLV320DAC3100_typedefs.h
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

#pragma once

/*!
 * @brief Headset detection debounce time options
 */
typedef enum {
  TLV320_DEBOUNCE_16MS = 0b000,  ///< 16ms debounce (2ms clock)
  TLV320_DEBOUNCE_32MS = 0b001,  ///< 32ms debounce (4ms clock)
  TLV320_DEBOUNCE_64MS = 0b010,  ///< 64ms debounce (8ms clock)
  TLV320_DEBOUNCE_128MS = 0b011, ///< 128ms debounce (16ms clock)
  TLV320_DEBOUNCE_256MS = 0b100, ///< 256ms debounce (32ms clock)
  TLV320_DEBOUNCE_512MS = 0b101, ///< 512ms debounce (64ms clock)
} tlv320_detect_debounce_t;

/*!
 * @brief Button press debounce time options
 */
typedef enum {
  TLV320_BTN_DEBOUNCE_0MS = 0b00,  ///< No debounce
  TLV320_BTN_DEBOUNCE_8MS = 0b01,  ///< 8ms debounce (1ms clock)
  TLV320_BTN_DEBOUNCE_16MS = 0b10, ///< 16ms debounce (2ms clock)
  TLV320_BTN_DEBOUNCE_32MS = 0b11, ///< 32ms debounce (4ms clock)
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
  TLV320_DAC_PATH_OFF = 0b00,     ///< DAC data path off
  TLV320_DAC_PATH_NORMAL = 0b01,  ///< Normal path (L->L or R->R)
  TLV320_DAC_PATH_SWAPPED = 0b10, ///< Swapped path (R->L or L->R)
  TLV320_DAC_PATH_MIXED = 0b11,   ///< Mixed L+R path
} tlv320_dac_path_t;

/*!
 * @brief DAC volume control soft stepping options
 */
typedef enum {
  TLV320_VOLUME_STEP_1SAMPLE = 0b00,  ///< One step per sample
  TLV320_VOLUME_STEP_2SAMPLE = 0b01,  ///< One step per two samples
  TLV320_VOLUME_STEP_DISABLED = 0b10, ///< Soft stepping disabled
} tlv320_volume_step_t;

/*!
 * @brief DAC volume control configuration options
 */
typedef enum {
  TLV320_VOL_INDEPENDENT = 0b00,   ///< Left and right channels independent
  TLV320_VOL_LEFT_TO_RIGHT = 0b01, ///< Left follows right volume
  TLV320_VOL_RIGHT_TO_LEFT = 0b10, ///< Right follows left volume
} tlv320_vol_control_t;

/*!
 * @brief Clock source options for CODEC_CLKIN
 */
typedef enum {
  TLV320DAC3100_CODEC_CLKIN_MCLK = 0b00,  ///< MCLK pin is the source
  TLV320DAC3100_CODEC_CLKIN_BCLK = 0b01,  ///< BCLK pin is the source
  TLV320DAC3100_CODEC_CLKIN_GPIO1 = 0b10, ///< GPIO1 pin is the source
  TLV320DAC3100_CODEC_CLKIN_PLL = 0b11,   ///< PLL_CLK pin is the source
} tlv320dac3100_codec_clkin_t;

/*!
 * @brief Clock source options for PLL_CLKIN
 */
typedef enum {
  TLV320DAC3100_PLL_CLKIN_MCLK = 0b00,  ///< MCLK pin is the source
  TLV320DAC3100_PLL_CLKIN_BCLK = 0b01,  ///< BCLK pin is the source
  TLV320DAC3100_PLL_CLKIN_GPIO1 = 0b10, ///< GPIO1 pin is the source
  TLV320DAC3100_PLL_CLKIN_DIN = 0b11    ///< DIN pin is the source
} tlv320dac3100_pll_clkin_t;

/*!
 * @brief Clock divider input source options
 */
typedef enum {
  TLV320DAC3100_CDIV_CLKIN_MCLK = 0b000, ///< MCLK (device pin)
  TLV320DAC3100_CDIV_CLKIN_BCLK = 0b001, ///< BCLK (device pin)
  TLV320DAC3100_CDIV_CLKIN_DIN =
      0b010, ///< DIN (for systems where DAC is not required)
  TLV320DAC3100_CDIV_CLKIN_PLL = 0b011, ///< PLL_CLK (generated on-chip)
  TLV320DAC3100_CDIV_CLKIN_DAC =
      0b100, ///< DAC_CLK (DAC DSP clock - generated on-chip)
  TLV320DAC3100_CDIV_CLKIN_DAC_MOD = 0b101, ///< DAC_MOD_CLK (generated on-chip)
} tlv320dac3100_cdiv_clkin_t;

/*!
 * @brief Data length for I2S interface
 */
typedef enum {
  TLV320DAC3100_DATA_LEN_16 = 0b00, ///< 16 bits
  TLV320DAC3100_DATA_LEN_20 = 0b01, ///< 20 bits
  TLV320DAC3100_DATA_LEN_24 = 0b10, ///< 24 bits
  TLV320DAC3100_DATA_LEN_32 = 0b11, ///< 32 bits
} tlv320dac3100_data_len_t;

/*!
 * @brief Data format for I2S interface
 */
typedef enum {
  TLV320DAC3100_FORMAT_I2S = 0b00, ///< I2S format
  TLV320DAC3100_FORMAT_DSP = 0b01, ///< DSP format
  TLV320DAC3100_FORMAT_RJF = 0b10, ///< Right justified format
  TLV320DAC3100_FORMAT_LJF = 0b11, ///< Left justified format
} tlv320dac3100_format_t;

/*!
 * @brief GPIO1 pin mode options
 */
typedef enum {
  TLV320_GPIO1_DISABLED =
      0b0000, ///< GPIO1 disabled (input and output buffers powered down)
  TLV320_GPIO1_INPUT_MODE =
      0b0001, ///< Input mode (secondary BCLK/WCLK/DIN input or ClockGen)
  TLV320_GPIO1_GPI = 0b0010,      ///< General-purpose input
  TLV320_GPIO1_GPO = 0b0011,      ///< General-purpose output
  TLV320_GPIO1_CLKOUT = 0b0100,   ///< CLKOUT output
  TLV320_GPIO1_INT1 = 0b0101,     ///< INT1 output
  TLV320_GPIO1_INT2 = 0b0110,     ///< INT2 output
  TLV320_GPIO1_BCLK_OUT = 0b1000, ///< Secondary BCLK output for codec interface
  TLV320_GPIO1_WCLK_OUT = 0b1001, ///< Secondary WCLK output for codec interface
} tlv320_gpio1_mode_t;

/*!
 * @brief DIN pin mode options
 */
typedef enum {
  TLV320_DIN_DISABLED = 0b00, ///< DIN disabled (input buffer powered down)
  TLV320_DIN_ENABLED = 0b01,  ///< DIN enabled (for codec interface/ClockGen)
  TLV320_DIN_GPI = 0b10,      ///< DIN used as general-purpose input
} tlv320_din_mode_t;

/*!
 * @brief Volume ADC hysteresis options
 */
typedef enum {
  TLV320_VOL_HYST_NONE = 0b00, ///< No hysteresis
  TLV320_VOL_HYST_1BIT = 0b01, ///< ±1 bit hysteresis
  TLV320_VOL_HYST_2BIT = 0b10, ///< ±2 bit hysteresis
} tlv320_vol_hyst_t;

/*!
 * @brief Volume ADC throughput rates
 */
typedef enum {
  TLV320_VOL_RATE_15_625HZ = 0b000, ///< 15.625 Hz (MCLK) or 10.68 Hz (RC)
  TLV320_VOL_RATE_31_25HZ = 0b001,  ///< 31.25 Hz (MCLK) or 21.35 Hz (RC)
  TLV320_VOL_RATE_62_5HZ = 0b010,   ///< 62.5 Hz (MCLK) or 42.71 Hz (RC)
  TLV320_VOL_RATE_125HZ = 0b011,    ///< 125 Hz (MCLK) or 85.2 Hz (RC)
  TLV320_VOL_RATE_250HZ = 0b100,    ///< 250 Hz (MCLK) or 170 Hz (RC)
  TLV320_VOL_RATE_500HZ = 0b101,    ///< 500 Hz (MCLK) or 340 Hz (RC)
  TLV320_VOL_RATE_1KHZ = 0b110,     ///< 1 kHz (MCLK) or 680 Hz (RC)
  TLV320_VOL_RATE_2KHZ = 0b111,     ///< 2 kHz (MCLK) or 1.37 kHz (RC)
} tlv320_vol_rate_t;

/*!
 * @brief Headphone common-mode settings
 */
typedef enum {
  TLV320_HP_COMMON_1_35V = 0b00, ///< Common-mode voltage 1.35V
  TLV320_HP_COMMON_1_50V = 0b01, ///< Common-mode voltage 1.50V
  TLV320_HP_COMMON_1_65V = 0b10, ///< Common-mode voltage 1.65V
  TLV320_HP_COMMON_1_80V = 0b11, ///< Common-mode voltage 1.80V
} tlv320_hp_common_t;

/*!
 * @brief Headphone driver power-on time options
 */
typedef enum {
  TLV320_HP_TIME_0US = 0b0000,   ///< 0 microseconds
  TLV320_HP_TIME_15US = 0b0001,  ///< 15.3 microseconds
  TLV320_HP_TIME_153US = 0b0010, ///< 153 microseconds
  TLV320_HP_TIME_1_5MS = 0b0011, ///< 1.53 milliseconds
  TLV320_HP_TIME_15MS = 0b0100,  ///< 15.3 milliseconds
  TLV320_HP_TIME_76MS = 0b0101,  ///< 76.2 milliseconds
  TLV320_HP_TIME_153MS = 0b0110, ///< 153 milliseconds
  TLV320_HP_TIME_304MS = 0b0111, ///< 304 milliseconds
  TLV320_HP_TIME_610MS = 0b1000, ///< 610 milliseconds
  TLV320_HP_TIME_1_2S = 0b1001,  ///< 1.22 seconds
  TLV320_HP_TIME_3S = 0b1010,    ///< 3.04 seconds
  TLV320_HP_TIME_6S = 0b1011,    ///< 6.1 seconds
} tlv320_hp_time_t;

/*!
 * @brief Headphone driver ramp-up step time options
 */
typedef enum {
  TLV320_RAMP_0MS = 0b00, ///< 0 milliseconds
  TLV320_RAMP_1MS = 0b01, ///< 0.98 milliseconds
  TLV320_RAMP_2MS = 0b10, ///< 1.95 milliseconds
  TLV320_RAMP_4MS = 0b11, ///< 3.9 milliseconds
} tlv320_ramp_time_t;

/*!
 * @brief Speaker power-up wait time options
 */
typedef enum {
  TLV320_SPK_WAIT_0MS = 0b000,  ///< 0 milliseconds
  TLV320_SPK_WAIT_3MS = 0b001,  ///< 3.04 milliseconds
  TLV320_SPK_WAIT_7MS = 0b010,  ///< 7.62 milliseconds
  TLV320_SPK_WAIT_12MS = 0b011, ///< 12.2 milliseconds
  TLV320_SPK_WAIT_15MS = 0b100, ///< 15.3 milliseconds
  TLV320_SPK_WAIT_19MS = 0b101, ///< 19.8 milliseconds
  TLV320_SPK_WAIT_24MS = 0b110, ///< 24.4 milliseconds
  TLV320_SPK_WAIT_30MS = 0b111, ///< 30.5 milliseconds
} tlv320_spk_wait_t;

/*!
 * @brief DAC output routing options
 */
typedef enum {
  TLV320_DAC_ROUTE_NONE = 0b00,  ///< DAC not routed
  TLV320_DAC_ROUTE_MIXER = 0b01, ///< DAC routed to mixer amplifier
  TLV320_DAC_ROUTE_HP = 0b10,    ///< DAC routed directly to HP driver
} tlv320_dac_route_t;

/*!
 * @brief MICBIAS voltage options
 */
typedef enum {
  TLV320_MICBIAS_OFF = 0b00,  ///< MICBIAS powered down
  TLV320_MICBIAS_2V = 0b01,   ///< MICBIAS = 2V
  TLV320_MICBIAS_2_5V = 0b10, ///< MICBIAS = 2.5V
  TLV320_MICBIAS_AVDD = 0b11, ///< MICBIAS = AVDD
} tlv320_micbias_volt_t;

/*!
 * @brief Speaker amplifier gain options
 */
typedef enum {
  TLV320_SPK_GAIN_6DB = 0b00,  ///< 6 dB gain
  TLV320_SPK_GAIN_12DB = 0b01, ///< 12 dB gain
  TLV320_SPK_GAIN_18DB = 0b10, ///< 18 dB gain
  TLV320_SPK_GAIN_24DB = 0b11, ///< 24 dB gain
} tlv320_spk_gain_t;

/*!
 * @brief BCLK source settings
 */
typedef enum {
  TLV320DAC3100_BCLK_SRC_DAC_CLK = 0,
  TLV320DAC3100_BCLK_SRC_DAC_MOD_CLK = 1,
} tlv320dac3100_bclk_src_t;
