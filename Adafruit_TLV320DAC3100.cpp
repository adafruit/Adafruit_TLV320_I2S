/*!
 * @file Adafruit_TLV320DAC3100.cpp
 *
 * @mainpage TI TLV320DAC3100 Stereo DAC with Headphone Amplifier
 *
 * @section intro_sec Introduction
 *
 * This is a library for the TI TLV320DAC3100 Stereo DAC with Headphone
 * Amplifier
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text above must be included in any redistribution
 */

#include "Adafruit_TLV320DAC3100.h"

/*!
 *    @brief  Instantiates a new TLV320DAC3100 class
 */
Adafruit_TLV320DAC3100::Adafruit_TLV320DAC3100() {}

/*!
 * @brief Initialize the DAC and verify communication
 * @param i2c_address The I2C address of the device
 * @param wire The I2C bus to use
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::begin(uint8_t i2c_address, TwoWire *wire) {
  if (i2c_dev) {
    delete i2c_dev;
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return reset();
}

/*!
 * @brief Set the current register page
 * @param page The page number to set (0-255)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setPage(uint8_t page) {
  Adafruit_BusIO_Register page_reg(i2c_dev, TLV320DAC3100_REG_PAGE_SELECT);
  return page_reg.write(page);
}

/*!
 * @brief Get the current register page
 * @return The current page number (0-255)
 */
uint8_t Adafruit_TLV320DAC3100::getPage(void) {
  Adafruit_BusIO_Register page_reg(i2c_dev, TLV320DAC3100_REG_PAGE_SELECT);
  uint8_t page = 0;
  page_reg.read(&page);
  return page;
}

/*!
 * @brief Perform a software reset of the chip
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::reset(void) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register reset_reg(i2c_dev, TLV320DAC3100_REG_RESET);
  Adafruit_BusIO_RegisterBits reset_bits(&reset_reg, 1, 0); // 1 bit, shift 0

  if (!reset_bits.write(1))
    return false;

  delay(10);

  return (!reset_bits.read());
}

/*!
 * @brief Check if the chip is in an over-temperature condition
 * @return true: overtemp condition exists, false: temperature is OK
 */
bool Adafruit_TLV320DAC3100::isOvertemperature(void) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register ot_reg(i2c_dev, TLV320DAC3100_REG_OT_FLAG);
  Adafruit_BusIO_RegisterBits ot_bits(&ot_reg, 1, 1); // 1 bit, shift 1

  return !ot_bits.read(); // invert since 0 = overtemp
}

/*!
 * @brief Set the PLL clock input source
 * @param clkin The clock input source to use
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setPLLClockInput(tlv320dac3100_pll_clkin_t clkin) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register clk_reg(i2c_dev, TLV320DAC3100_REG_CLOCK_MUX1);
  Adafruit_BusIO_RegisterBits pll_clkin(&clk_reg, 2,
                                        2); // 2 bits, starting at bit 2

  return pll_clkin.write(clkin);
}

/*!
 * @brief Get the current PLL clock input source
 * @return The current PLL clock input source
 */
tlv320dac3100_pll_clkin_t Adafruit_TLV320DAC3100::getPLLClockInput(void) {
  if (!setPage(0)) {
    return TLV320DAC3100_PLL_CLKIN_MCLK; // return default on failure
  }

  Adafruit_BusIO_Register clk_reg(i2c_dev, TLV320DAC3100_REG_CLOCK_MUX1);
  Adafruit_BusIO_RegisterBits pll_clkin(&clk_reg, 2,
                                        2); // 2 bits, starting at bit 2

  return (tlv320dac3100_pll_clkin_t)pll_clkin.read();
}

/*!
 * @brief Set the CODEC clock input source
 * @param clkin The clock input source to use
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setCodecClockInput(
    tlv320dac3100_codec_clkin_t clkin) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register clk_reg(i2c_dev, TLV320DAC3100_REG_CLOCK_MUX1);
  Adafruit_BusIO_RegisterBits codec_clkin(&clk_reg, 2,
                                          0); // 2 bits, starting at bit 0

  return codec_clkin.write(clkin);
}

/*!
 * @brief Get the current CODEC clock input source
 * @return The current CODEC clock input source
 */
tlv320dac3100_codec_clkin_t Adafruit_TLV320DAC3100::getCodecClockInput(void) {
  if (!setPage(0)) {
    return TLV320DAC3100_CODEC_CLKIN_MCLK; // return default on failure
  }

  Adafruit_BusIO_Register clk_reg(i2c_dev, TLV320DAC3100_REG_CLOCK_MUX1);
  Adafruit_BusIO_RegisterBits codec_clkin(&clk_reg, 2,
                                          0); // 2 bits, starting at bit 0

  return (tlv320dac3100_codec_clkin_t)codec_clkin.read();
}

/*!
 * @brief Set the PLL power state
 * @param on true to power on, false to power off
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::powerPLL(bool on) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register pll_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_PR);
  Adafruit_BusIO_RegisterBits pll_power(&pll_reg, 1, 7); // bit 7

  return pll_power.write(on);
}

/*!
 * @brief Get the PLL power state
 * @return true if PLL is powered on, false if powered off
 */
bool Adafruit_TLV320DAC3100::isPLLpowered(void) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register pll_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_PR);
  Adafruit_BusIO_RegisterBits pll_power(&pll_reg, 1, 7); // bit 7

  return pll_power.read();
}

/*!
 * @brief Set the PLL P, R, J, and D values
 * @param P PLL P value (1-8)
 * @param R PLL R value (1-16)
 * @param J PLL J value (1-63)
 * @param D PLL D value (0-9999)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setPLLValues(uint8_t P, uint8_t R, uint8_t J,
                                          uint16_t D) {
  if (!setPage(0)) {
    return false;
  }

  // Validate all input ranges
  if (P < 1 || P > 8) {
    return false;
  }
  if (R < 1 || R > 16) {
    return false;
  }
  if (J < 1 || J > 63) {
    return false;
  }
  if (D > 9999) {
    return false;
  }

  // P & R register
  Adafruit_BusIO_Register pr_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_PR);
  Adafruit_BusIO_RegisterBits p_bits(&pr_reg, 3, 4); // bits 6:4
  Adafruit_BusIO_RegisterBits r_bits(&pr_reg, 4, 0); // bits 3:0

  if (!p_bits.write(P % 8))
    return false; // P values wrap at 8
  if (!r_bits.write(R % 16))
    return false; // R values wrap at 16

  // J register
  Adafruit_BusIO_Register j_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_J);
  Adafruit_BusIO_RegisterBits j_bits(&j_reg, 6, 0); // bits 5:0
  if (!j_bits.write(J))
    return false;

  // D MSB & LSB registers (14 bits total)
  Adafruit_BusIO_Register d_msb_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_D_MSB);
  Adafruit_BusIO_Register d_lsb_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_D_LSB);

  if (!d_msb_reg.write(D >> 8))
    return false;
  if (!d_lsb_reg.write(D & 0xFF))
    return false;

  return true;
}

/*!
 * @brief Get the PLL P, R, J, and D values
 * @param P Pointer to store P value (1-8), or NULL
 * @param R Pointer to store R value (1-16), or NULL
 * @param J Pointer to store J value (1-63), or NULL
 * @param D Pointer to store D value (0-9999), or NULL
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getPLLValues(uint8_t *P, uint8_t *R, uint8_t *J,
                                          uint16_t *D) {
  if (!setPage(0)) {
    return false;
  }

  // P & R register
  Adafruit_BusIO_Register pr_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_PR);
  Adafruit_BusIO_RegisterBits p_bits(&pr_reg, 3, 4); // bits 6:4
  Adafruit_BusIO_RegisterBits r_bits(&pr_reg, 4, 0); // bits 3:0

  if (P) {
    uint8_t p_val = p_bits.read();
    *P = (p_val == 0) ? 8 : p_val; // 0 represents 8
  }
  if (R) {
    uint8_t r_val = r_bits.read();
    *R = (r_val == 0) ? 16 : r_val; // 0 represents 16
  }

  // J register
  if (J) {
    Adafruit_BusIO_Register j_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_J);
    Adafruit_BusIO_RegisterBits j_bits(&j_reg, 6, 0); // bits 5:0
    *J = j_bits.read();
  }

  // D MSB & LSB registers (14 bits total)
  if (D) {
    Adafruit_BusIO_Register d_msb_reg(i2c_dev,
                                      TLV320DAC3100_REG_PLL_PROG_D_MSB);
    Adafruit_BusIO_Register d_lsb_reg(i2c_dev,
                                      TLV320DAC3100_REG_PLL_PROG_D_LSB);
    uint8_t msb, lsb;
    if (!d_msb_reg.read(&msb))
      return false;
    if (!d_lsb_reg.read(&lsb))
      return false;
    *D = ((uint16_t)msb << 8) | lsb;
  }

  return true;
}

/*!
 * @brief Set the NDAC value and enable/disable
 * @param enable True to enable NDAC, false to disable
 * @param val NDAC divider value (1-128)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setNDAC(bool enable, uint8_t val) {
  if (!setPage(0)) {
    return false;
  }

  // Validate input range
  if (val < 1 || val > 128) {
    return false;
  }

  Adafruit_BusIO_Register ndac_reg(i2c_dev, TLV320DAC3100_REG_NDAC);
  Adafruit_BusIO_RegisterBits ndac_en(&ndac_reg, 1, 7);  // enable bit
  Adafruit_BusIO_RegisterBits ndac_val(&ndac_reg, 7, 0); // value bits

  if (!ndac_en.write(enable))
    return false;
  if (!ndac_val.write(val % 128))
    return false; // 0 represents 128

  return true;
}

/*!
 * @brief Get the NDAC value and enabled state
 * @param enabled Pointer to store enabled state, or NULL
 * @param val Pointer to store NDAC value (1-128), or NULL
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getNDAC(bool *enabled, uint8_t *val) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register ndac_reg(i2c_dev, TLV320DAC3100_REG_NDAC);
  Adafruit_BusIO_RegisterBits ndac_en(&ndac_reg, 1, 7);  // enable bit
  Adafruit_BusIO_RegisterBits ndac_val(&ndac_reg, 7, 0); // value bits

  if (enabled) {
    *enabled = ndac_en.read();
  }

  if (val) {
    uint8_t v = ndac_val.read();
    *val = (v == 0) ? 128 : v;
  }

  return true;
}

/*!
 * @brief Set the MDAC value and enable/disable
 * @param enable True to enable MDAC, false to disable
 * @param val MDAC divider value (1-128)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setMDAC(bool enable, uint8_t val) {
  if (!setPage(0)) {
    return false;
  }

  // Validate input range
  if (val < 1 || val > 128) {
    return false;
  }

  Adafruit_BusIO_Register mdac_reg(i2c_dev, TLV320DAC3100_REG_MDAC);
  Adafruit_BusIO_RegisterBits mdac_en(&mdac_reg, 1, 7);  // enable bit
  Adafruit_BusIO_RegisterBits mdac_val(&mdac_reg, 7, 0); // value bits

  if (!mdac_en.write(enable))
    return false;
  if (!mdac_val.write(val % 128))
    return false; // 0 represents 128

  return true;
}

/*!
 * @brief Get the MDAC value and enabled state
 * @param enabled Pointer to store enabled state, or NULL
 * @param val Pointer to store MDAC value (1-128), or NULL
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getMDAC(bool *enabled, uint8_t *val) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register mdac_reg(i2c_dev, TLV320DAC3100_REG_MDAC);
  Adafruit_BusIO_RegisterBits mdac_en(&mdac_reg, 1, 7);  // enable bit
  Adafruit_BusIO_RegisterBits mdac_val(&mdac_reg, 7, 0); // value bits

  if (enabled) {
    *enabled = mdac_en.read();
  }

  if (val) {
    uint8_t v = mdac_val.read();
    *val = (v == 0) ? 128 : v;
  }

  return true;
}

/*!
 * @brief Set the DOSR value
 * @param val DOSR divider value (2-1024, except 1023)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setDOSR(uint16_t val) {
  if (!setPage(0)) {
    return false;
  }

  // Validate input range
  if (val < 2 || val > 1024 || val == 1023) {
    return false;
  }

  uint16_t dosr_val = val % 1024;

  Adafruit_BusIO_Register dosr_msb_reg(i2c_dev, TLV320DAC3100_REG_DOSR_MSB);
  Adafruit_BusIO_Register dosr_lsb_reg(i2c_dev, TLV320DAC3100_REG_DOSR_LSB);
  if (!dosr_msb_reg.write(dosr_val >> 8))
    return false;
  if (!dosr_lsb_reg.write(dosr_val & 0xFF))
    return false;

  return true;
}

/*!
 * @brief Get the DOSR value
 * @param val Pointer to store DOSR value (2-1024, except 1023)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getDOSR(uint16_t *val) {
  if (!setPage(0) || !val) {
    return false;
  }

  Adafruit_BusIO_Register dosr_msb_reg(i2c_dev, TLV320DAC3100_REG_DOSR_MSB);
  Adafruit_BusIO_Register dosr_lsb_reg(i2c_dev, TLV320DAC3100_REG_DOSR_LSB);
  uint8_t msb, lsb;
  if (!dosr_msb_reg.read(&msb))
    return false;
  if (!dosr_lsb_reg.read(&lsb))
    return false;
  uint16_t v = ((uint16_t)msb << 8) | lsb;
  *val = (v == 0) ? 1024 : v;

  return true;
}

/*!
 * @brief Set the clock divider input source
 * @param clkin The clock input source to use
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setClockDividerInput(
    tlv320dac3100_cdiv_clkin_t clkin) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register mux_reg(i2c_dev, TLV320DAC3100_REG_CLKOUT_MUX);
  Adafruit_BusIO_RegisterBits cdiv_bits(&mux_reg, 3,
                                        0); // 3 bits, starting at bit 0

  return cdiv_bits.write(clkin);
}

/*!
 * @brief Get the current clock divider input source
 * @return The current clock divider input source
 */
tlv320dac3100_cdiv_clkin_t Adafruit_TLV320DAC3100::getClockDividerInput(void) {
  if (!setPage(0)) {
    return TLV320DAC3100_CDIV_CLKIN_MCLK; // return default on failure
  }

  Adafruit_BusIO_Register mux_reg(i2c_dev, TLV320DAC3100_REG_CLKOUT_MUX);
  Adafruit_BusIO_RegisterBits cdiv_bits(&mux_reg, 3,
                                        3); // 3 bits, starting at bit 3

  return (tlv320dac3100_cdiv_clkin_t)cdiv_bits.read();
}

/*!
 * @brief Set the CLKOUT M divider value and enable/disable
 * @param enable True to enable CLKOUT_M, false to disable
 * @param val M divider value (1-128)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setCLKOUT_M(bool enable, uint8_t val) {
  if (!setPage(0)) {
    return false;
  }

  // Validate input range
  if (val < 1 || val > 128) {
    return false;
  }

  Adafruit_BusIO_Register m_reg(i2c_dev, TLV320DAC3100_REG_CLKOUT_M);
  Adafruit_BusIO_RegisterBits m_en(&m_reg, 1, 7);  // enable bit
  Adafruit_BusIO_RegisterBits m_val(&m_reg, 7, 0); // value bits

  if (!m_en.write(enable))
    return false;
  if (!m_val.write(val % 128))
    return false; // 0 represents 128

  return true;
}

/*!
 * @brief Get the CLKOUT M divider value and enabled state
 * @param enabled Pointer to store enabled state, or NULL
 * @param val Pointer to store M value (1-128), or NULL
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getCLKOUT_M(bool *enabled, uint8_t *val) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register m_reg(i2c_dev, TLV320DAC3100_REG_CLKOUT_M);
  Adafruit_BusIO_RegisterBits m_en(&m_reg, 1, 7);  // enable bit
  Adafruit_BusIO_RegisterBits m_val(&m_reg, 7, 0); // value bits

  if (enabled) {
    *enabled = m_en.read();
  }

  if (val) {
    uint8_t v = m_val.read();
    *val = (v == 0) ? 128 : v;
  }

  return true;
}

/*!
 * @brief Set the codec interface parameters
 * @param format Audio data format
 * @param len Word length
 * @param bclk_out Optional: true for BCLK output, false for input (default
 * false)
 * @param wclk_out Optional: true for WCLK output, false for input (default
 * false)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setCodecInterface(tlv320dac3100_format_t format,
                                               tlv320dac3100_data_len_t len,
                                               bool bclk_out, bool wclk_out) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register ctrl_reg(i2c_dev, TLV320DAC3100_REG_CODEC_IF_CTRL1);
  Adafruit_BusIO_RegisterBits format_bits(&ctrl_reg, 2, 6); // bits 7:6
  Adafruit_BusIO_RegisterBits len_bits(&ctrl_reg, 2, 4);    // bits 5:4
  Adafruit_BusIO_RegisterBits bclk_bits(&ctrl_reg, 1, 3);   // bit 3
  Adafruit_BusIO_RegisterBits wclk_bits(&ctrl_reg, 1, 2);   // bit 2

  if (!format_bits.write(format))
    return false;
  if (!len_bits.write(len))
    return false;
  if (!bclk_bits.write(bclk_out))
    return false;
  if (!wclk_bits.write(wclk_out))
    return false;

  return true;
}

/*!
 * @brief Get the codec interface parameters
 * @param format Pointer to store audio format, or NULL
 * @param len Pointer to store word length, or NULL
 * @param bclk_out Pointer to store BCLK direction, or NULL
 * @param wclk_out Pointer to store WCLK direction, or NULL
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getCodecInterface(tlv320dac3100_format_t *format,
                                               tlv320dac3100_data_len_t *len,
                                               bool *bclk_out, bool *wclk_out) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register ctrl_reg(i2c_dev, TLV320DAC3100_REG_CODEC_IF_CTRL1);
  Adafruit_BusIO_RegisterBits format_bits(&ctrl_reg, 2, 6); // bits 7:6
  Adafruit_BusIO_RegisterBits len_bits(&ctrl_reg, 2, 4);    // bits 5:4
  Adafruit_BusIO_RegisterBits bclk_bits(&ctrl_reg, 1, 3);   // bit 3
  Adafruit_BusIO_RegisterBits wclk_bits(&ctrl_reg, 1, 2);   // bit 2

  if (format) {
    *format = (tlv320dac3100_format_t)format_bits.read();
  }
  if (len) {
    *len = (tlv320dac3100_data_len_t)len_bits.read();
  }
  if (bclk_out) {
    *bclk_out = bclk_bits.read();
  }
  if (wclk_out) {
    *wclk_out = wclk_bits.read();
  }

  return true;
}

/*!
 * @brief Set the BCLK data slot offset
 * @param offset BCLK offset value (0-255)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setBCLKoffset(uint8_t offset) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register offset_reg(i2c_dev,
                                     TLV320DAC3100_REG_DATA_SLOT_OFFSET);
  return offset_reg.write(offset);
}

/*!
 * @brief Get the BCLK data slot offset
 * @param offset Pointer to store offset value (0-255)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getBCLKoffset(uint8_t *offset) {
  if (!setPage(0) || !offset) {
    return false;
  }

  Adafruit_BusIO_Register offset_reg(i2c_dev,
                                     TLV320DAC3100_REG_DATA_SLOT_OFFSET);
  return offset_reg.read(offset);
}

/*!
 * @brief Set the BCLK N divider value and enable/disable
 * @param enable True to enable BCLK_N, false to disable
 * @param val N divider value (1-128)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setBCLK_N(bool enable, uint8_t val) {
  if (!setPage(0)) {
    return false;
  }

  // Validate input range
  if (val < 1 || val > 128) {
    return false;
  }

  Adafruit_BusIO_Register n_reg(i2c_dev, TLV320DAC3100_REG_BCLK_N);
  Adafruit_BusIO_RegisterBits n_en(&n_reg, 1, 7);  // enable bit
  Adafruit_BusIO_RegisterBits n_val(&n_reg, 7, 0); // value bits

  if (!n_en.write(enable))
    return false;
  if (!n_val.write(val % 128))
    return false; // 0 represents 128

  return true;
}

/*!
 * @brief Get the BCLK N divider value and enabled state
 * @param enabled Pointer to store enabled state, or NULL
 * @param val Pointer to store N value (1-128), or NULL
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getBCLK_N(bool *enabled, uint8_t *val) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register n_reg(i2c_dev, TLV320DAC3100_REG_BCLK_N);
  Adafruit_BusIO_RegisterBits n_en(&n_reg, 1, 7);  // enable bit
  Adafruit_BusIO_RegisterBits n_val(&n_reg, 7, 0); // value bits

  if (enabled) {
    *enabled = n_en.read();
  }

  if (val) {
    uint8_t v = n_val.read();
    *val = (v == 0) ? 128 : v;
  }

  return true;
}

/*!
 * @brief Configure the BCLK interface settings
 *
 * @param invert_bclk Whether to invert the BCLK signal
 * @param active_when_powered_down Keep BCLK and WCLK active when codec powered
 * down
 * @param source Select BCLK input source (DAC_CLK or DAC_MOD_CLK)
 * @return true: success, false: failure
 */
bool Adafruit_TLV320DAC3100::setBCLKConfig(bool invert_bclk,
                                           bool active_when_powered_down,
                                           tlv320dac3100_bclk_src_t source) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register reg =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BCLK_CTRL2);
  Adafruit_BusIO_RegisterBits bclk_invert =
      Adafruit_BusIO_RegisterBits(&reg, 1, 3);
  Adafruit_BusIO_RegisterBits bclk_active_pd =
      Adafruit_BusIO_RegisterBits(&reg, 1, 2);
  Adafruit_BusIO_RegisterBits bclk_src =
      Adafruit_BusIO_RegisterBits(&reg, 2, 0);

  if (!bclk_invert.write(invert_bclk) ||
      !bclk_active_pd.write(active_when_powered_down) ||
      !bclk_src.write(source)) {
    return false;
  }

  return true;
}

/*!
 * @brief Get the current BCLK interface settings
 *
 * @param invert_bclk Pointer to store BCLK inversion state
 * @param active_when_powered_down Pointer to store BCLK active state during
 * power down
 * @param source Pointer to store BCLK source setting
 * @return true: success, false: failure
 */
bool Adafruit_TLV320DAC3100::getBCLKConfig(bool *invert_bclk,
                                           bool *active_when_powered_down,
                                           tlv320dac3100_bclk_src_t *source) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register reg =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BCLK_CTRL2);
  Adafruit_BusIO_RegisterBits bclk_invert =
      Adafruit_BusIO_RegisterBits(&reg, 1, 3);
  Adafruit_BusIO_RegisterBits bclk_active_pd =
      Adafruit_BusIO_RegisterBits(&reg, 1, 2);
  Adafruit_BusIO_RegisterBits bclk_src =
      Adafruit_BusIO_RegisterBits(&reg, 2, 0);

  if (invert_bclk) {
    *invert_bclk = bclk_invert.read();
  }
  if (active_when_powered_down) {
    *active_when_powered_down = bclk_active_pd.read();
  }
  if (source) {
    *source = (tlv320dac3100_bclk_src_t)bclk_src.read();
  }

  return true;
}

/*!
 * @brief Get the DAC and output driver status flags
 * @param left_dac_powered Pointer to store Left DAC power status, or NULL
 * @param hpl_powered Pointer to store HPL driver power status, or NULL
 * @param left_classd_powered Pointer to store Left Class-D power status, or
 * NULL
 * @param right_dac_powered Pointer to store Right DAC power status, or NULL
 * @param hpr_powered Pointer to store HPR driver power status, or NULL
 * @param right_classd_powered Pointer to store Right Class-D power status, or
 * NULL
 * @param left_pga_gain_ok Pointer to store Left PGA gain match status, or NULL
 * @param right_pga_gain_ok Pointer to store Right PGA gain match status, or
 * NULL
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getDACFlags(
    bool *left_dac_powered, bool *hpl_powered, bool *left_classd_powered,
    bool *right_dac_powered, bool *hpr_powered, bool *right_classd_powered,
    bool *left_pga_gain_ok, bool *right_pga_gain_ok) {
  if (!setPage(0)) {
    return false;
  }

  // Read first flag register
  Adafruit_BusIO_Register flag_reg(i2c_dev, TLV320DAC3100_REG_DAC_FLAG);
  Adafruit_BusIO_RegisterBits ldac_bit(&flag_reg, 1, 7);    // bit 7
  Adafruit_BusIO_RegisterBits hpl_bit(&flag_reg, 1, 5);     // bit 5
  Adafruit_BusIO_RegisterBits lclassd_bit(&flag_reg, 1, 4); // bit 4
  Adafruit_BusIO_RegisterBits rdac_bit(&flag_reg, 1, 3);    // bit 3
  Adafruit_BusIO_RegisterBits hpr_bit(&flag_reg, 1, 1);     // bit 1
  Adafruit_BusIO_RegisterBits rclassd_bit(&flag_reg, 1, 0); // bit 0

  // Read second flag register
  Adafruit_BusIO_Register flag2_reg(i2c_dev, TLV320DAC3100_REG_DAC_FLAG2);
  Adafruit_BusIO_RegisterBits lpga_bit(&flag2_reg, 1, 4); // bit 4
  Adafruit_BusIO_RegisterBits rpga_bit(&flag2_reg, 1, 0); // bit 0

  if (left_dac_powered) {
    *left_dac_powered = ldac_bit.read();
  }
  if (hpl_powered) {
    *hpl_powered = hpl_bit.read();
  }
  if (left_classd_powered) {
    *left_classd_powered = lclassd_bit.read();
  }
  if (right_dac_powered) {
    *right_dac_powered = rdac_bit.read();
  }
  if (hpr_powered) {
    *hpr_powered = hpr_bit.read();
  }
  if (right_classd_powered) {
    *right_classd_powered = rclassd_bit.read();
  }
  if (left_pga_gain_ok) {
    *left_pga_gain_ok = lpga_bit.read();
  }
  if (right_pga_gain_ok) {
    *right_pga_gain_ok = rpga_bit.read();
  }

  return true;
}

/*!
 * @brief Configure the INT1 interrupt sources
 *
 * @param headset_detect Enable headset detection interrupt
 * @param button_press Enable button press detection interrupt
 * @param dac_drc Enable DAC DRC signal power interrupt
 * @param agc_noise Enable DAC data overflow interrupt
 * @param over_current Enable short circuit interrupt
 * @param multiple_pulse If true, INT1 generates multiple pulses until flag read
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setInt1Source(bool headset_detect,
                                           bool button_press, bool dac_drc,
                                           bool agc_noise, bool over_current,
                                           bool multiple_pulse) {
  uint8_t int_config = 0;

  if (!setPage(0)) {
    return false;
  }

  if (headset_detect)
    int_config |= (1 << 7);
  if (button_press)
    int_config |= (1 << 6);
  if (dac_drc)
    int_config |= (1 << 5);
  if (over_current)
    int_config |= (1 << 3);
  if (agc_noise)
    int_config |= (1 << 2);
  if (multiple_pulse)
    int_config |= (1 << 0);

  Adafruit_BusIO_Register int1_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_INT1_CTRL);

  return int1_ctrl.write(int_config);
}

/*!
 * @brief Configure the INT2 interrupt sources
 *
 * @param headset_detect Enable headset detection interrupt
 * @param button_press Enable button press detection interrupt
 * @param dac_drc Enable DAC DRC signal power interrupt
 * @param agc_noise Enable DAC data overflow interrupt
 * @param over_current Enable short circuit interrupt
 * @param multiple_pulse If true, INT2 generates multiple pulses until flag read
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setInt2Source(bool headset_detect,
                                           bool button_press, bool dac_drc,
                                           bool agc_noise, bool over_current,
                                           bool multiple_pulse) {
  uint8_t int_config = 0;

  if (!setPage(0)) {
    return false;
  }

  if (headset_detect)
    int_config |= (1 << 7);
  if (button_press)
    int_config |= (1 << 6);
  if (dac_drc)
    int_config |= (1 << 5);
  if (over_current)
    int_config |= (1 << 3);
  if (agc_noise)
    int_config |= (1 << 2);
  if (multiple_pulse)
    int_config |= (1 << 0);

  Adafruit_BusIO_Register int2_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_INT2_CTRL);

  return int2_ctrl.write(int_config);
}

/*!
 * @brief Set the GPIO1 pin mode
 *
 * @param mode The GPIO1 pin mode/function
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setGPIO1Mode(tlv320_gpio1_mode_t mode) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register gpio1_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_GPIO1_CTRL);

  Adafruit_BusIO_RegisterBits gpio1_mode =
      Adafruit_BusIO_RegisterBits(&gpio1_ctrl, 4, 2);

  return gpio1_mode.write(mode);
}

/*!
 * @brief Set the GPIO1 output value (only valid in GPO mode)
 *
 * @param value The output value to set
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setGPIO1Output(bool value) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register gpio1_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_GPIO1_CTRL);

  Adafruit_BusIO_RegisterBits gpio1_out =
      Adafruit_BusIO_RegisterBits(&gpio1_ctrl, 1, 0);

  return gpio1_out.write(value);
}

/*!
 * @brief Get the current GPIO1 mode configuration
 *
 * @return Current GPIO1 mode setting
 */
tlv320_gpio1_mode_t Adafruit_TLV320DAC3100::getGPIO1Mode() {
  setPage(0);

  Adafruit_BusIO_Register gpio1_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_GPIO1_CTRL);

  Adafruit_BusIO_RegisterBits gpio1_mode =
      Adafruit_BusIO_RegisterBits(&gpio1_ctrl, 4, 2);

  return (tlv320_gpio1_mode_t)gpio1_mode.read();
}

/*!
 * @brief Get the current GPIO1 input value
 *
 * @return Current GPIO1 input state
 */
bool Adafruit_TLV320DAC3100::getGPIO1Input() {
  Adafruit_BusIO_Register gpio1_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_GPIO1_CTRL);

  Adafruit_BusIO_RegisterBits gpio1_in =
      Adafruit_BusIO_RegisterBits(&gpio1_ctrl, 1, 1);

  return gpio1_in.read();
}

/*!
 * @brief Set the DIN pin mode
 * @param mode The DIN pin mode/function
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setDINMode(tlv320_din_mode_t mode) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register din_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DIN_CTRL);

  Adafruit_BusIO_RegisterBits din_mode =
      Adafruit_BusIO_RegisterBits(&din_ctrl, 2, 1);

  return din_mode.write(mode);
}

/*!
 * @brief Get the current DIN pin mode
 * @return Current DIN mode setting
 */
tlv320_din_mode_t Adafruit_TLV320DAC3100::getDINMode() {
  if (!setPage(0)) {
    return TLV320_DIN_DISABLED;
  }

  Adafruit_BusIO_Register din_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DIN_CTRL);

  Adafruit_BusIO_RegisterBits din_mode =
      Adafruit_BusIO_RegisterBits(&din_ctrl, 2, 1);

  return (tlv320_din_mode_t)din_mode.read();
}

/*!
 * @brief Get the current DIN pin input value
 * @return Current DIN input state (true or false)
 */
bool Adafruit_TLV320DAC3100::getDINInput() {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register din_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DIN_CTRL);

  Adafruit_BusIO_RegisterBits din_in =
      Adafruit_BusIO_RegisterBits(&din_ctrl, 1, 0);

  return din_in.read();
}

/*!
 * @brief Set the DAC Processing Block selection (PRB_P1 through PRB_P25)
 *
 * @param block_number Processing block number (1-25)
 * @return true: success false: failure or invalid input
 */
bool Adafruit_TLV320DAC3100::setDACProcessingBlock(uint8_t block_number) {
  if (block_number < 1 || block_number > 25) {
    return false;
  }

  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register dac_block =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_PRB);

  Adafruit_BusIO_RegisterBits block_select =
      Adafruit_BusIO_RegisterBits(&dac_block, 5, 0);

  return block_select.write(block_number);
}

/*!
 * @brief Get the current DAC Processing Block selection
 *
 * @return Current block number (1-25), 0 if read fails
 */
uint8_t Adafruit_TLV320DAC3100::getDACProcessingBlock() {
  if (!setPage(0)) {
    return 0;
  }

  Adafruit_BusIO_Register dac_block =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_PRB);

  Adafruit_BusIO_RegisterBits block_select =
      Adafruit_BusIO_RegisterBits(&dac_block, 5, 0);

  return block_select.read();
}

/*!
 * @brief Configure the DAC data path settings
 *
 * @param left_dac_on Power up left DAC
 * @param right_dac_on Power up right DAC
 * @param left_path Left channel data path configuration
 * @param right_path Right channel data path configuration
 * @param volume_step Volume control soft stepping configuration
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setDACDataPath(bool left_dac_on, bool right_dac_on,
                                            tlv320_dac_path_t left_path,
                                            tlv320_dac_path_t right_path,
                                            tlv320_volume_step_t volume_step) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register dac_path =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_DATAPATH);

  Adafruit_BusIO_RegisterBits left_power =
      Adafruit_BusIO_RegisterBits(&dac_path, 1, 7);
  Adafruit_BusIO_RegisterBits right_power =
      Adafruit_BusIO_RegisterBits(&dac_path, 1, 6);
  Adafruit_BusIO_RegisterBits left_data =
      Adafruit_BusIO_RegisterBits(&dac_path, 2, 4);
  Adafruit_BusIO_RegisterBits right_data =
      Adafruit_BusIO_RegisterBits(&dac_path, 2, 2);
  Adafruit_BusIO_RegisterBits vol_step =
      Adafruit_BusIO_RegisterBits(&dac_path, 2, 0);

  if (!left_power.write(left_dac_on))
    return false;
  if (!right_power.write(right_dac_on))
    return false;
  if (!left_data.write(left_path))
    return false;
  if (!right_data.write(right_path))
    return false;
  return vol_step.write(volume_step);
}

/*!
 * @brief Get the current DAC data path configuration
 *
 * @param left_dac_on Pointer to store left DAC power state
 * @param right_dac_on Pointer to store right DAC power state
 * @param left_path Pointer to store left channel data path configuration
 * @param right_path Pointer to store right channel data path configuration
 * @param volume_step Pointer to store volume stepping configuration
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getDACDataPath(bool *left_dac_on,
                                            bool *right_dac_on,
                                            tlv320_dac_path_t *left_path,
                                            tlv320_dac_path_t *right_path,
                                            tlv320_volume_step_t *volume_step) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register dac_path =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_DATAPATH);

  Adafruit_BusIO_RegisterBits left_power =
      Adafruit_BusIO_RegisterBits(&dac_path, 1, 7);
  Adafruit_BusIO_RegisterBits right_power =
      Adafruit_BusIO_RegisterBits(&dac_path, 1, 6);
  Adafruit_BusIO_RegisterBits left_data =
      Adafruit_BusIO_RegisterBits(&dac_path, 2, 4);
  Adafruit_BusIO_RegisterBits right_data =
      Adafruit_BusIO_RegisterBits(&dac_path, 2, 2);
  Adafruit_BusIO_RegisterBits vol_step =
      Adafruit_BusIO_RegisterBits(&dac_path, 2, 0);

  if (left_dac_on)
    *left_dac_on = left_power.read();
  if (right_dac_on)
    *right_dac_on = right_power.read();
  if (left_path)
    *left_path = (tlv320_dac_path_t)left_data.read();
  if (right_path)
    *right_path = (tlv320_dac_path_t)right_data.read();
  if (volume_step)
    *volume_step = (tlv320_volume_step_t)vol_step.read();

  return true;
}

/*!
 * @brief Configure the DAC volume control settings
 *
 * @param left_mute Mute left DAC channel
 * @param right_mute Mute right DAC channel
 * @param control Volume control configuration
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setDACVolumeControl(bool left_mute,
                                                 bool right_mute,
                                                 tlv320_vol_control_t control) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register vol_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_VOL_CTRL);

  Adafruit_BusIO_RegisterBits left_mute_bit =
      Adafruit_BusIO_RegisterBits(&vol_ctrl, 1, 3);
  Adafruit_BusIO_RegisterBits right_mute_bit =
      Adafruit_BusIO_RegisterBits(&vol_ctrl, 1, 2);
  Adafruit_BusIO_RegisterBits vol_ctrl_bits =
      Adafruit_BusIO_RegisterBits(&vol_ctrl, 2, 0);

  if (!left_mute_bit.write(left_mute))
    return false;
  if (!right_mute_bit.write(right_mute))
    return false;
  return vol_ctrl_bits.write(control);
}

/*!
 * @brief Get the current DAC volume control configuration
 *
 * @param left_mute Pointer to store left channel mute state
 * @param right_mute Pointer to store right channel mute state
 * @param control Pointer to store volume control configuration
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getDACVolumeControl(
    bool *left_mute, bool *right_mute, tlv320_vol_control_t *control) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register vol_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_VOL_CTRL);

  Adafruit_BusIO_RegisterBits left_mute_bit =
      Adafruit_BusIO_RegisterBits(&vol_ctrl, 1, 3);
  Adafruit_BusIO_RegisterBits right_mute_bit =
      Adafruit_BusIO_RegisterBits(&vol_ctrl, 1, 2);
  Adafruit_BusIO_RegisterBits vol_ctrl_bits =
      Adafruit_BusIO_RegisterBits(&vol_ctrl, 2, 0);

  if (left_mute)
    *left_mute = left_mute_bit.read();
  if (right_mute)
    *right_mute = right_mute_bit.read();
  if (control)
    *control = (tlv320_vol_control_t)vol_ctrl_bits.read();

  return true;
}

/*!
 * @brief Set DAC channel volume in dB
 *
 * @param right_channel true for right channel, false for left channel
 * @param dB Volume in dB (-63.5 to +24 dB)
 * @return true: success false: failure or invalid input
 */
bool Adafruit_TLV320DAC3100::setChannelVolume(bool right_channel, float dB) {
  if (!setPage(0)) {
    return false;
  }

  // Constrain input to valid range
  if (dB > 24.0)
    dB = 24.0;
  if (dB < -63.5)
    dB = -63.5;

  int8_t reg_val;
  reg_val = dB * 2;

  // Check for reserved values
  if ((reg_val == 0x80) || (reg_val > 0x30)) {
    return false;
  }

  Adafruit_BusIO_Register vol_ctrl = Adafruit_BusIO_Register(
      i2c_dev,
      right_channel ? TLV320DAC3100_REG_DAC_RVOL : TLV320DAC3100_REG_DAC_LVOL);

  return vol_ctrl.write(reg_val);
}

/*!
 * @brief Get DAC channel volume in dB
 *
 * @param right_channel true for right channel, false for left channel
 * @return Current volume in dB
 */
float Adafruit_TLV320DAC3100::getChannelVolume(bool right_channel) {
  if (!setPage(0)) {
    return 0.0;
  }

  Adafruit_BusIO_Register vol_ctrl = Adafruit_BusIO_Register(
      i2c_dev,
      right_channel ? TLV320DAC3100_REG_DAC_RVOL : TLV320DAC3100_REG_DAC_LVOL);

  uint8_t reg_val;
  if (!vol_ctrl.read(&reg_val)) {
    return 0.0;
  }

  // Convert from two's complement to signed value
  int8_t steps;
  if (reg_val & 0x80) {
    steps = (int8_t)(reg_val - 256);
  } else {
    steps = (int8_t)reg_val;
  }

  // Convert half-steps to dB
  return steps * 0.5;
}

/*!
 * @brief Configure headset detection settings
 *
 * @param enable Enable headset detection
 * @param detect_debounce Headset detection debounce time
 * @param button_debounce Button press debounce time
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setHeadsetDetect(
    bool enable, tlv320_detect_debounce_t detect_debounce,
    tlv320_button_debounce_t button_debounce) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register headset =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HEADSET_DETECT);

  Adafruit_BusIO_RegisterBits enable_bit =
      Adafruit_BusIO_RegisterBits(&headset, 1, 7);
  Adafruit_BusIO_RegisterBits detect_debounce_bits =
      Adafruit_BusIO_RegisterBits(&headset, 3, 2);
  Adafruit_BusIO_RegisterBits button_debounce_bits =
      Adafruit_BusIO_RegisterBits(&headset, 2, 0);

  if (!enable_bit.write(enable))
    return false;
  if (!detect_debounce_bits.write(detect_debounce))
    return false;
  return button_debounce_bits.write(button_debounce);
}

/*!
 * @brief Get current headset detection status
 *
 * @return Current headset status
 */
tlv320_headset_status_t Adafruit_TLV320DAC3100::getHeadsetStatus(void) {
  if (!setPage(0)) {
    return TLV320_HEADSET_NONE;
  }

  Adafruit_BusIO_Register headset =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HEADSET_DETECT);

  Adafruit_BusIO_RegisterBits status_bits =
      Adafruit_BusIO_RegisterBits(&headset, 2, 5);

  return (tlv320_headset_status_t)status_bits.read();
}

/*!
 * @brief Set beep volume for left and right channels
 *
 * @param left_dB Left channel volume (+2 to -61 dB)
 * @param right_dB Right channel volume (+2 to -61 dB), if -100 matches left
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setBeepVolume(int8_t left_dB, int8_t right_dB) {
  if (!setPage(0)) {
    return false;
  }

  // Constrain volumes
  if (left_dB > 2)
    left_dB = 2;
  if (left_dB < -61)
    left_dB = -61;

  // Convert to register values (2dB = 0x00, -61dB = 0x3F)
  uint8_t left_reg = (uint8_t)(-left_dB + 2);

  bool match_volumes = (right_dB == -100);
  if (match_volumes) {
    right_dB = left_dB;
  }
  if (right_dB > 2)
    right_dB = 2;
  if (right_dB < -61)
    right_dB = -61;
  uint8_t right_reg = (uint8_t)(-right_dB + 2);

  Adafruit_BusIO_Register beep_l =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_L);
  Adafruit_BusIO_Register beep_r =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_R);

  Adafruit_BusIO_RegisterBits left_vol =
      Adafruit_BusIO_RegisterBits(&beep_l, 6, 0);
  Adafruit_BusIO_RegisterBits right_vol =
      Adafruit_BusIO_RegisterBits(&beep_r, 6, 0);
  Adafruit_BusIO_RegisterBits vol_ctrl =
      Adafruit_BusIO_RegisterBits(&beep_r, 2, 6);

  if (!left_vol.write(left_reg))
    return false;
  if (!right_vol.write(right_reg))
    return false;

  // Set volume control mode
  return vol_ctrl.write(match_volumes ? 0b01 : 0b00);
}

/*!
 * @brief Set beep length in samples
 *
 * @param samples Number of samples to generate beep (24-bit value)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setBeepLength(uint32_t samples) {
  if (!setPage(0)) {
    return false;
  }

  // Limit to 24-bit value
  samples &= 0x00FFFFFF;

  Adafruit_BusIO_Register beep_msb =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_LEN_MSB);
  Adafruit_BusIO_Register beep_mid =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_LEN_MID);
  Adafruit_BusIO_Register beep_lsb =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_LEN_LSB);

  if (!beep_msb.write(samples >> 16))
    return false;
  if (!beep_mid.write(samples >> 8))
    return false;
  return beep_lsb.write(samples);
}

/*!
 * @brief Set beep sine and cosine values for frequency generation
 *
 * @param sin_val 16-bit sine value for sin(2p � fin / fS)
 * @param cos_val 16-bit cosine value for cos(2p � fin / fS)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setBeepSinCos(uint16_t sin_val, uint16_t cos_val) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register beep_sin_msb =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_SIN_MSB);
  Adafruit_BusIO_Register beep_sin_lsb =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_SIN_LSB);
  Adafruit_BusIO_Register beep_cos_msb =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_COS_MSB);
  Adafruit_BusIO_Register beep_cos_lsb =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_COS_LSB);

  if (!beep_sin_msb.write(sin_val >> 8))
    return false;
  if (!beep_sin_lsb.write(sin_val & 0xFF))
    return false;
  if (!beep_cos_msb.write(cos_val >> 8))
    return false;
  return beep_cos_lsb.write(cos_val & 0xFF);
}

/*!
 * @brief Configure the Volume/MicDet pin ADC
 *
 * @param pin_control Enable pin control of DAC volume
 * @param use_mclk Use MCLK instead of internal RC oscillator
 * @param hysteresis ADC hysteresis setting
 * @param rate ADC sampling rate
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::configVolADC(bool pin_control, bool use_mclk,
                                          tlv320_vol_hyst_t hysteresis,
                                          tlv320_vol_rate_t rate) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register vol_adc =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_VOL_ADC_CTRL);

  Adafruit_BusIO_RegisterBits pin_ctrl =
      Adafruit_BusIO_RegisterBits(&vol_adc, 1, 7);
  Adafruit_BusIO_RegisterBits clk_src =
      Adafruit_BusIO_RegisterBits(&vol_adc, 1, 6);
  Adafruit_BusIO_RegisterBits hyst_bits =
      Adafruit_BusIO_RegisterBits(&vol_adc, 2, 4);
  Adafruit_BusIO_RegisterBits rate_bits =
      Adafruit_BusIO_RegisterBits(&vol_adc, 3, 0);

  if (!pin_ctrl.write(pin_control))
    return false;
  if (!clk_src.write(use_mclk))
    return false;
  if (!hyst_bits.write(hysteresis))
    return false;
  return rate_bits.write(rate);
}

/*!
 * @brief Read the current volume from the Volume ADC in dB
 *
 * @return Current volume in dB (+18 to -63 dB)
 */
float Adafruit_TLV320DAC3100::readVolADCdB(void) {
  if (!setPage(0)) {
    return 0.0;
  }

  Adafruit_BusIO_Register vol_adc =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_VOL_ADC_READ);

  Adafruit_BusIO_RegisterBits vol_bits =
      Adafruit_BusIO_RegisterBits(&vol_adc, 7, 0);

  uint8_t raw_val = vol_bits.read();

  // Check for reserved value
  if (raw_val == 0x7F) {
    return 0.0;
  }

  // Convert register value to dB
  // 0x00 = +18dB, 0x24 = 0dB, 0x7E = -63dB
  if (raw_val <= 0x24) {
    // Positive or zero dB range
    return 18.0 - (raw_val * 0.5);
  } else {
    // Negative dB range
    return -(raw_val - 0x24) * 0.5;
  }
}

/*!
 * @brief Configure speaker reset behavior on short circuit detection
 *
 * @param reset True to reset speaker on short circuit, false to remain
 * unchanged
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::resetSpeakerOnSCD(bool reset) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register err_ctl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HP_SPK_ERR_CTL);

  Adafruit_BusIO_RegisterBits spk_reset =
      Adafruit_BusIO_RegisterBits(&err_ctl, 1, 1);

  return spk_reset.write(!reset); // Register is inverse of parameter
}

/*!
 * @brief Configure headphone reset behavior on short circuit detection
 *
 * @param reset True to reset headphone on short circuit, false to remain
 * unchanged
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::resetHeadphoneOnSCD(bool reset) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register err_ctl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HP_SPK_ERR_CTL);

  Adafruit_BusIO_RegisterBits hp_reset =
      Adafruit_BusIO_RegisterBits(&err_ctl, 1, 0);

  return hp_reset.write(!reset); // Register is inverse of parameter
}

/*!
 * @brief Enable or disable the Class-D speaker amplifier
 *
 * @param en True to enable, false to disable
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::enableSpeaker(bool en) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register spk_amp =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_SPK_AMP);

  Adafruit_BusIO_RegisterBits spk_en =
      Adafruit_BusIO_RegisterBits(&spk_amp, 1, 7);

  return spk_en.write(en);
}

/*!
 * @brief Check if the Class-D speaker amplifier is enabled
 *
 * @return true if enabled, false if disabled
 */
bool Adafruit_TLV320DAC3100::speakerEnabled(void) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register spk_amp =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_SPK_AMP);

  Adafruit_BusIO_RegisterBits spk_en =
      Adafruit_BusIO_RegisterBits(&spk_amp, 1, 7);

  return spk_en.read();
}

/*!
 * @brief Check if speaker short circuit is detected
 *
 * @return true if short circuit detected, false if not
 */
bool Adafruit_TLV320DAC3100::isSpeakerShorted(void) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register spk_amp =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_SPK_AMP);

  Adafruit_BusIO_RegisterBits short_detect =
      Adafruit_BusIO_RegisterBits(&spk_amp, 1, 0);

  return short_detect.read();
}

/*!
 * @brief Configure headphone pop removal settings
 *
 * @param wait_for_powerdown Wait for amp powerdown before DAC powerdown
 * @param powerup_time Driver power-on time
 * @param ramp_time Driver ramp-up step time
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::configureHeadphonePop(
    bool wait_for_powerdown, tlv320_hp_time_t powerup_time,
    tlv320_ramp_time_t ramp_time) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register hp_pop =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HP_POP);

  Adafruit_BusIO_RegisterBits powerdown_mode =
      Adafruit_BusIO_RegisterBits(&hp_pop, 1, 7);
  Adafruit_BusIO_RegisterBits power_time =
      Adafruit_BusIO_RegisterBits(&hp_pop, 4, 3);
  Adafruit_BusIO_RegisterBits ramp_step =
      Adafruit_BusIO_RegisterBits(&hp_pop, 2, 1);

  if (!powerdown_mode.write(wait_for_powerdown))
    return false;
  if (!power_time.write(powerup_time))
    return false;
  return ramp_step.write(ramp_time);
}

/*!
 * @brief Set speaker power-up wait time
 *
 * @param wait_time Speaker power-up wait duration
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setSpeakerWaitTime(tlv320_spk_wait_t wait_time) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register pga_ramp =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_PGA_RAMP);

  Adafruit_BusIO_RegisterBits wait_bits =
      Adafruit_BusIO_RegisterBits(&pga_ramp, 3, 4);

  return wait_bits.write(wait_time);
}

/*!
 * @brief Configure DAC and analog input routing
 *
 * @param left_dac Left DAC routing
 * @param right_dac Right DAC routing
 * @param left_ain1 Route AIN1 to left mixer
 * @param left_ain2 Route AIN2 to left mixer
 * @param right_ain2 Route AIN2 to right mixer
 * @param hpl_routed_to_hpr Route HPL output to HPR input (differential mode)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::configureAnalogInputs(
    tlv320_dac_route_t left_dac, tlv320_dac_route_t right_dac, bool left_ain1,
    bool left_ain2, bool right_ain2, bool hpl_routed_to_hpr) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register routing =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_OUT_ROUTING);
  Adafruit_BusIO_RegisterBits left_dac_route =
      Adafruit_BusIO_RegisterBits(&routing, 2, 6);
  Adafruit_BusIO_RegisterBits left_ain1_route =
      Adafruit_BusIO_RegisterBits(&routing, 1, 5);
  Adafruit_BusIO_RegisterBits left_ain2_route =
      Adafruit_BusIO_RegisterBits(&routing, 1, 4);
  Adafruit_BusIO_RegisterBits right_dac_route =
      Adafruit_BusIO_RegisterBits(&routing, 2, 2);
  Adafruit_BusIO_RegisterBits right_ain2_route =
      Adafruit_BusIO_RegisterBits(&routing, 1, 1);
  Adafruit_BusIO_RegisterBits hpl_to_hpr =
      Adafruit_BusIO_RegisterBits(&routing, 1, 0);

  if (!left_dac_route.write(left_dac))
    return false;
  if (!left_ain1_route.write(left_ain1))
    return false;
  if (!left_ain2_route.write(left_ain2))
    return false;
  if (!right_dac_route.write(right_dac))
    return false;
  if (!right_ain2_route.write(right_ain2))
    return false;
  if (!hpl_to_hpr.write(hpl_routed_to_hpr))
    return false;

  return true;
}

/*!
 * @brief Set HPL analog volume control
 *
 * @param route_enabled Enable routing to HPL output driver
 * @param gain Gain value (0-127, see datasheet Table 6-24 for dB values)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setHPLVolume(bool route_enabled, uint8_t gain) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register hpl_vol =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HPL_VOL);

  Adafruit_BusIO_RegisterBits route =
      Adafruit_BusIO_RegisterBits(&hpl_vol, 1, 7);
  Adafruit_BusIO_RegisterBits vol = Adafruit_BusIO_RegisterBits(&hpl_vol, 7, 0);

  // Constrain gain to valid range
  if (gain > 0x7F)
    gain = 0x7F;

  if (!route.write(route_enabled))
    return false;
  return vol.write(gain);
}

/*!
 * @brief Set HPR analog volume control
 *
 * @param route_enabled Enable routing to HPR output driver
 * @param gain Gain value (0-127, see datasheet Table 6-24 for dB values)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setHPRVolume(bool route_enabled, uint8_t gain) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register hpr_vol =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HPR_VOL);

  Adafruit_BusIO_RegisterBits route =
      Adafruit_BusIO_RegisterBits(&hpr_vol, 1, 7);
  Adafruit_BusIO_RegisterBits vol = Adafruit_BusIO_RegisterBits(&hpr_vol, 7, 0);

  if (gain > 0x7F)
    gain = 0x7F;

  if (!route.write(route_enabled))
    return false;
  return vol.write(gain);
}

/*!
 * @brief Set Speaker analog volume control
 *
 * @param route_enabled Enable routing to class-D output driver
 * @param gain Gain value (0-127, see datasheet Table 6-24 for dB values)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setSPKVolume(bool route_enabled, uint8_t gain) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register spk_vol =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_SPK_VOL);

  Adafruit_BusIO_RegisterBits route =
      Adafruit_BusIO_RegisterBits(&spk_vol, 1, 7);
  Adafruit_BusIO_RegisterBits vol = Adafruit_BusIO_RegisterBits(&spk_vol, 7, 0);

  if (gain > 0x7F)
    gain = 0x7F;

  if (!route.write(route_enabled))
    return false;
  return vol.write(gain);
}

/*!
 * @brief Configure HPL driver PGA settings
 *
 * @param gain_db PGA gain (0-9 dB)
 * @param unmute True to unmute, false to mute
 * @return true: success false: failure or invalid gain
 */
bool Adafruit_TLV320DAC3100::configureHPL_PGA(uint8_t gain_db, bool unmute) {
  if (!setPage(1)) {
    return false;
  }

  if (gain_db > 9) {
    return false;
  }

  Adafruit_BusIO_Register hpl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HPL_DRIVER);

  Adafruit_BusIO_RegisterBits gain = Adafruit_BusIO_RegisterBits(&hpl, 4, 3);
  Adafruit_BusIO_RegisterBits mute = Adafruit_BusIO_RegisterBits(&hpl, 1, 2);

  if (!gain.write(gain_db))
    return false;
  return mute.write(unmute);
}

/*!
 * @brief Configure HPR driver PGA settings
 *
 * @param gain_db PGA gain (0-9 dB)
 * @param unmute True to unmute, false to mute
 * @return true: success false: failure or invalid gain
 */
bool Adafruit_TLV320DAC3100::configureHPR_PGA(uint8_t gain_db, bool unmute) {
  if (!setPage(1)) {
    return false;
  }

  if (gain_db > 9) {
    return false;
  }

  Adafruit_BusIO_Register hpr =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HPR_DRIVER);

  Adafruit_BusIO_RegisterBits gain = Adafruit_BusIO_RegisterBits(&hpr, 4, 3);
  Adafruit_BusIO_RegisterBits mute = Adafruit_BusIO_RegisterBits(&hpr, 1, 2);

  if (!gain.write(gain_db))
    return false;
  return mute.write(unmute);
}

/*!
 * @brief Configure Speaker driver settings
 *
 * @param gain Output stage gain setting
 * @param unmute True to unmute, false to mute
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::configureSPK_PGA(tlv320_spk_gain_t gain,
                                              bool unmute) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register spk =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_SPK_DRIVER);

  Adafruit_BusIO_RegisterBits gain_bits =
      Adafruit_BusIO_RegisterBits(&spk, 2, 3);
  Adafruit_BusIO_RegisterBits mute = Adafruit_BusIO_RegisterBits(&spk, 1, 2);

  if (!gain_bits.write(gain))
    return false;
  return mute.write(unmute);
}

/*!
 * @brief Check if all programmed gains have been applied to HPL
 *
 * @return true if gains applied, false if still ramping
 */
bool Adafruit_TLV320DAC3100::isHPLGainApplied(void) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register hpl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HPL_DRIVER);

  Adafruit_BusIO_RegisterBits applied = Adafruit_BusIO_RegisterBits(&hpl, 1, 0);

  return applied.read();
}

/*!
 * @brief Check if all programmed gains have been applied to HPR
 *
 * @return true if gains applied, false if still ramping
 */
bool Adafruit_TLV320DAC3100::isHPRGainApplied(void) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register hpr =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HPR_DRIVER);

  Adafruit_BusIO_RegisterBits applied = Adafruit_BusIO_RegisterBits(&hpr, 1, 0);

  return applied.read();
}

/*!
 * @brief Check if all programmed gains have been applied to Speaker
 *
 * @return true if gains applied, false if still ramping
 */
bool Adafruit_TLV320DAC3100::isSPKGainApplied(void) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register spk =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_SPK_DRIVER);

  Adafruit_BusIO_RegisterBits applied = Adafruit_BusIO_RegisterBits(&spk, 1, 0);

  return applied.read();
}

/*!
 * @brief Configure headphone outputs as line-out
 *
 * @param left Configure left channel as line-out
 * @param right Configure right channel as line-out
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::headphoneLineout(bool left, bool right) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register hp_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HP_DRIVER_CTRL);

  Adafruit_BusIO_RegisterBits left_line =
      Adafruit_BusIO_RegisterBits(&hp_ctrl, 1, 2);
  Adafruit_BusIO_RegisterBits right_line =
      Adafruit_BusIO_RegisterBits(&hp_ctrl, 1, 1);

  if (!left_line.write(left))
    return false;
  return right_line.write(right);
}

/*!
 * @brief Configure MICBIAS settings
 *
 * @param power_down Enable software power down
 * @param always_on Keep MICBIAS on even without headset
 * @param voltage MICBIAS voltage setting
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::configMicBias(bool power_down, bool always_on,
                                           tlv320_micbias_volt_t voltage) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register micbias =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_MICBIAS);

  Adafruit_BusIO_RegisterBits powerdown =
      Adafruit_BusIO_RegisterBits(&micbias, 1, 7);
  Adafruit_BusIO_RegisterBits force_on =
      Adafruit_BusIO_RegisterBits(&micbias, 1, 3);
  Adafruit_BusIO_RegisterBits volt =
      Adafruit_BusIO_RegisterBits(&micbias, 2, 0);

  if (!powerdown.write(power_down))
    return false;
  if (!force_on.write(always_on))
    return false;
  return volt.write(voltage);
}

/*!
 * @brief Configure analog input common mode connections
 *
 * @param ain1_cm Connect AIN1 to common mode when unused
 * @param ain2_cm Connect AIN2 to common mode when unused
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setInputCommonMode(bool ain1_cm, bool ain2_cm) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register input_cm =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_INPUT_CM);

  Adafruit_BusIO_RegisterBits ain1 =
      Adafruit_BusIO_RegisterBits(&input_cm, 1, 7);
  Adafruit_BusIO_RegisterBits ain2 =
      Adafruit_BusIO_RegisterBits(&input_cm, 1, 6);

  if (!ain1.write(ain1_cm))
    return false;
  return ain2.write(ain2_cm);
}

/*!
 * @brief Configure programmable delay timer clock source and divider
 *
 * @param use_mclk True to use external MCLK, false for internal oscillator
 * @param divider Clock divider (1-127, or 0 for 128)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::configDelayDivider(bool use_mclk,
                                                uint8_t divider) {
  if (!setPage(3)) {
    return false;
  }

  Adafruit_BusIO_Register timer =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_TIMER_MCLK_DIV);

  Adafruit_BusIO_RegisterBits clk_src =
      Adafruit_BusIO_RegisterBits(&timer, 1, 7);
  Adafruit_BusIO_RegisterBits div = Adafruit_BusIO_RegisterBits(&timer, 7, 0);

  if (!clk_src.write(use_mclk))
    return false;
  return div.write(divider);
}

/*!
 * @brief Configure headphone driver settings
 *
 * @param left_powered Power up left headphone driver
 * @param right_powered Power up right headphone driver
 * @param common Common-mode voltage setting
 * @param powerDownOnSCD Power down on short circuit (vs. current limiting)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::configureHeadphoneDriver(bool left_powered,
                                                      bool right_powered,
                                                      tlv320_hp_common_t common,
                                                      bool powerDownOnSCD) {
  if (!setPage(1)) {
    return false;
  }

  Adafruit_BusIO_Register hp_ctrl =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_HP_DRIVERS);

  Adafruit_BusIO_RegisterBits left_power =
      Adafruit_BusIO_RegisterBits(&hp_ctrl, 1, 7);
  Adafruit_BusIO_RegisterBits right_power =
      Adafruit_BusIO_RegisterBits(&hp_ctrl, 1, 6);
  Adafruit_BusIO_RegisterBits common_mode =
      Adafruit_BusIO_RegisterBits(&hp_ctrl, 2, 3);
  Adafruit_BusIO_RegisterBits scd_mode =
      Adafruit_BusIO_RegisterBits(&hp_ctrl, 1, 1);

  uint8_t reg_val = 0x04; // bit 2 must be 1
  if (!left_power.write(left_powered))
    return false;
  if (!right_power.write(right_powered))
    return false;
  if (!common_mode.write(common))
    return false;
  return scd_mode.write(powerDownOnSCD);
}

/*!
 * @brief Calculate and set PLL values for desired frequency
 *
 * @param mclk_freq MCLK input frequency in Hz
 * @param desired_freq Desired PLL output frequency in Hz
 * @param max_error Maximum allowed error (default 0.001 = 0.1%)
 * @return true: success false: failure or invalid frequency
 */
bool Adafruit_TLV320DAC3100::configurePLL(uint32_t mclk_freq,
                                          uint32_t desired_freq,
                                          float max_error) {
  float ratio = (float)desired_freq / mclk_freq;
  float best_error = 1.0; // 100% error to start
  uint8_t best_P = 1, best_R = 1, best_J = 0;
  uint16_t best_D = 0;

  // Try different P & R values
  for (uint8_t P = 1; P <= 8; P++) {
    for (uint8_t R = 1; R <= 16; R++) {
      float J_float = ratio * P * R;
      if (J_float > 63)
        continue;

      uint8_t J = (uint8_t)J_float;
      uint16_t D = (uint16_t)((J_float - J) * 2048);
      if (D > 2047)
        continue;

      // Calculate actual frequency ratio this would give
      float actual_ratio = (float)(J + (float)D / 2048.0) / (P * R);
      float error = fabs(actual_ratio - ratio) / ratio;

      // If this is better than our best so far, save it
      if (error < best_error) {
        best_error = error;
        best_P = P;
        best_R = R;
        best_J = J;
        best_D = D;

        // If we're within acceptable error, use these values
        if (error <= max_error) {
          return setPLLValues(P, R, J, D);
        }
      }
    }
  }

  // If we got here, use the best values we found
  if (best_error < 0.1) { // Accept up to 10% error rather than totally fail
    return setPLLValues(best_P, best_R, best_J, best_D);
  }

  return false; // No acceptable values found
}

/*!
 * @brief Validate PLL configuration parameters
 *
 * @param P PLL divider (1-8)
 * @param R PLL multiplier (1-16)
 * @param J PLL multiplier (1-63)
 * @param D PLL fractional multiplier (0-2047)
 * @param pll_clkin Input clock frequency in Hz
 * @return true if configuration is valid, false if not
 */
bool Adafruit_TLV320DAC3100::validatePLLConfig(uint8_t P, uint8_t R, uint8_t J,
                                               uint16_t D, float pll_clkin) {
  float pll_in_div_p = pll_clkin / P;
  float jd = J + (float)D / 2048.0;
  float pll_out = (pll_clkin * jd * R) / P;

  if (D == 0) {
    // Check D=0 constraints
    if (pll_in_div_p < 512000 || pll_in_div_p > 20000000) {
      return false; // PLL_CLKIN/P out of range (512kHz-20MHz)
    }
    if (pll_out < 80000000 || pll_out > 110000000) {
      return false; // PLL output out of range (80-110MHz)
    }
    if ((R * J) < 4 || (R * J) > 259) {
      return false; // R*J out of range (4-259)
    }
  } else {
    // Check D?0 constraints
    if (pll_in_div_p < 10000000 || pll_in_div_p > 20000000) {
      return false; // PLL_CLKIN/P out of range (10-20MHz)
    }
    if (pll_out < 80000000 || pll_out > 110000000) {
      return false; // PLL output out of range (80-110MHz)
    }
    if (R != 1) {
      return false; // R must be 1 when D?0
    }
  }
  return true;
}

/*!
 * @brief Read a register value from the device
 * @param page The page number to access
 * @param reg The register address within the page
 * @return The value of the register
 */
uint8_t Adafruit_TLV320DAC3100::readRegister(uint8_t page, uint8_t reg) {
  setPage(page);
  Adafruit_BusIO_Register dac_reg = Adafruit_BusIO_Register(i2c_dev, reg);
  return dac_reg.read();
}

/*!
 * @brief Read the IRQ flags register
 *
 * @param sticky true to read the sticky version (0x2C), false to read the
 * non-sticky version (0x2E)
 * @return Byte containing the IRQ flags (use TLV320DAC3100_IRQ_* defines to
 * interpret)
 */
uint8_t Adafruit_TLV320DAC3100::readIRQflags(bool sticky) {
  if (!setPage(0)) {
    return 0;
  }

  // Use appropriate register based on sticky flag
  uint8_t reg =
      sticky ? TLV320DAC3100_REG_IRQ_FLAGS_STICKY : TLV320DAC3100_REG_IRQ_FLAGS;

  Adafruit_BusIO_Register flags_reg = Adafruit_BusIO_Register(i2c_dev, reg);
  uint8_t flags = 0;
  flags_reg.read(&flags);

  return flags;
}

/*!
 * @brief Enable or disable the beep generator
 *
 * @param enable true to enable beep, false to disable
 * @return true: success, false: failure
 */
bool Adafruit_TLV320DAC3100::enableBeep(bool enable) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register reg =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_L);
  Adafruit_BusIO_RegisterBits beep_enable =
      Adafruit_BusIO_RegisterBits(&reg, 1, 7);

  return beep_enable.write(enable);
}

/*!
 * @brief Check if beep generator is currently active
 *
 * @return true if beeping, false if not or on error
 */
bool Adafruit_TLV320DAC3100::isBeeping(void) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register reg =
      Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_BEEP_L);
  Adafruit_BusIO_RegisterBits beep_enable =
      Adafruit_BusIO_RegisterBits(&reg, 1, 7);

  return beep_enable.read();
}

/*!
 * @brief Configure beep generator for a specific frequency and duration
 *
 * @param frequency Desired frequency in Hz (must be < sample_rate/4)
 * @param duration_ms Length of beep in milliseconds
 * @param sample_rate Sample rate in Hz (default 48000)
 * @return true: success, false: frequency too high for sample rate or write
 * failed
 */
bool Adafruit_TLV320DAC3100::configureBeepTone(float frequency,
                                               uint32_t duration_ms,
                                               uint32_t sample_rate) {
  // Check frequency limit (Fs/4)
  if (frequency >= (sample_rate / 4.0f)) {
    return false;
  }

  // Calculate sine and cosine coefficients
  float angle = 2.0f * PI * frequency / sample_rate;
  uint16_t sin_val = round(sin(angle) * 32767.0f);
  uint16_t cos_val = round(cos(angle) * 32767.0f);

  // Convert ms to samples
  uint32_t length = (duration_ms * sample_rate) / 1000;
  if (length > 0x00FFFFFF) {
    length = 0x00FFFFFF;
  }

  return (setBeepSinCos(sin_val, cos_val) && setBeepLength(length));
}
