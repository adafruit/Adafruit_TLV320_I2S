/*!
 * @file Adafruit_TLV320DAC3100.cpp
 *
 * @mainpage TI TLV320DAC3100 Stereo DAC with Headphone Amplifier
 *
 * @section intro_sec Introduction
 *
 * This is a library for the TI TLV320DAC3100 Stereo DAC with Headphone Amplifier
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
  
  return reset_bits.write(1);
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
  Adafruit_BusIO_RegisterBits pll_clkin(&clk_reg, 2, 2); // 2 bits, starting at bit 2
  
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
  Adafruit_BusIO_RegisterBits pll_clkin(&clk_reg, 2, 2); // 2 bits, starting at bit 2
  
  return (tlv320dac3100_pll_clkin_t)pll_clkin.read();
}

/*!
 * @brief Set the CODEC clock input source
 * @param clkin The clock input source to use
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setCodecClockInput(tlv320dac3100_codec_clkin_t clkin) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register clk_reg(i2c_dev, TLV320DAC3100_REG_CLOCK_MUX1);
  Adafruit_BusIO_RegisterBits codec_clkin(&clk_reg, 2, 0); // 2 bits, starting at bit 0
  
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
  Adafruit_BusIO_RegisterBits codec_clkin(&clk_reg, 2, 0); // 2 bits, starting at bit 0
  
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
bool Adafruit_TLV320DAC3100::setPLLValues(uint8_t P, uint8_t R, uint8_t J, uint16_t D) {
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
  
  if (!p_bits.write(P % 8)) return false;  // P values wrap at 8
  if (!r_bits.write(R % 16)) return false; // R values wrap at 16

  // J register
  Adafruit_BusIO_Register j_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_J);
  Adafruit_BusIO_RegisterBits j_bits(&j_reg, 6, 0); // bits 5:0
  if (!j_bits.write(J)) return false;

  // D MSB & LSB registers (14 bits total)
  Adafruit_BusIO_Register d_msb_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_D_MSB);
  Adafruit_BusIO_Register d_lsb_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_D_LSB);
  
  if (!d_msb_reg.write(D >> 8)) return false;
  if (!d_lsb_reg.write(D & 0xFF)) return false;

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
bool Adafruit_TLV320DAC3100::getPLLValues(uint8_t *P, uint8_t *R, uint8_t *J, uint16_t *D) {
  if (!setPage(0)) {
    return false;
  }

  // P & R register
  Adafruit_BusIO_Register pr_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_PR);
  Adafruit_BusIO_RegisterBits p_bits(&pr_reg, 3, 4); // bits 6:4
  Adafruit_BusIO_RegisterBits r_bits(&pr_reg, 4, 0); // bits 3:0
  
  if (P) {
    uint8_t p_val = p_bits.read();
    *P = (p_val == 0) ? 8 : p_val;  // 0 represents 8
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
    Adafruit_BusIO_Register d_msb_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_D_MSB);
    Adafruit_BusIO_Register d_lsb_reg(i2c_dev, TLV320DAC3100_REG_PLL_PROG_D_LSB);
    uint8_t msb, lsb;
    if (!d_msb_reg.read(&msb)) return false;
    if (!d_lsb_reg.read(&lsb)) return false;
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
  
  if (!ndac_en.write(enable)) return false;
  if (!ndac_val.write(val % 128)) return false;  // 0 represents 128

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
  
  if (!mdac_en.write(enable)) return false;
  if (!mdac_val.write(val % 128)) return false;  // 0 represents 128

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
  if (!dosr_msb_reg.write(dosr_val >> 8)) return false;
  if (!dosr_lsb_reg.write(dosr_val & 0xFF)) return false;

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
  if (!dosr_msb_reg.read(&msb)) return false;
  if (!dosr_lsb_reg.read(&lsb)) return false;
  uint16_t v = ((uint16_t)msb << 8) | lsb;
  *val = (v == 0) ? 1024 : v;

  return true;
}


/*!
 * @brief Set the clock divider input source
 * @param clkin The clock input source to use
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setClockDividerInput(tlv320dac3100_cdiv_clkin_t clkin) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register mux_reg(i2c_dev, TLV320DAC3100_REG_CLKOUT_MUX);
  Adafruit_BusIO_RegisterBits cdiv_bits(&mux_reg, 3, 3); // 3 bits, starting at bit 3
  
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
  Adafruit_BusIO_RegisterBits cdiv_bits(&mux_reg, 3, 3); // 3 bits, starting at bit 3
  
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
  
  if (!m_en.write(enable)) return false;
  if (!m_val.write(val % 128)) return false;  // 0 represents 128

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
 * @param bclk_out Optional: true for BCLK output, false for input (default false)
 * @param wclk_out Optional: true for WCLK output, false for input (default false)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setCodecInterface(tlv320dac3100_format_t format, 
                                              tlv320dac3100_data_len_t len,
                                              bool bclk_out = false,
                                              bool wclk_out = false) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register ctrl_reg(i2c_dev, TLV320DAC3100_REG_CODEC_IF_CTRL1);
  Adafruit_BusIO_RegisterBits format_bits(&ctrl_reg, 2, 6);  // bits 7:6
  Adafruit_BusIO_RegisterBits len_bits(&ctrl_reg, 2, 4);     // bits 5:4
  Adafruit_BusIO_RegisterBits bclk_bits(&ctrl_reg, 1, 3);    // bit 3
  Adafruit_BusIO_RegisterBits wclk_bits(&ctrl_reg, 1, 2);    // bit 2

  if (!format_bits.write(format)) return false;
  if (!len_bits.write(len)) return false;
  if (!bclk_bits.write(bclk_out)) return false;
  if (!wclk_bits.write(wclk_out)) return false;

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
                                              bool *bclk_out,
                                              bool *wclk_out) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register ctrl_reg(i2c_dev, TLV320DAC3100_REG_CODEC_IF_CTRL1);
  Adafruit_BusIO_RegisterBits format_bits(&ctrl_reg, 2, 6);  // bits 7:6
  Adafruit_BusIO_RegisterBits len_bits(&ctrl_reg, 2, 4);     // bits 5:4
  Adafruit_BusIO_RegisterBits bclk_bits(&ctrl_reg, 1, 3);    // bit 3
  Adafruit_BusIO_RegisterBits wclk_bits(&ctrl_reg, 1, 2);    // bit 2

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

// In header:
#define TLV320DAC3100_REG_DATA_SLOT_OFFSET    0x1C    ///< Data-slot offset register

  bool setBCLKoffset(uint8_t offset);
  bool getBCLKoffset(uint8_t *offset);

// In cpp:
/*!
 * @brief Set the BCLK data slot offset
 * @param offset BCLK offset value (0-255)
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::setBCLKoffset(uint8_t offset) {
  if (!setPage(0)) {
    return false;
  }

  Adafruit_BusIO_Register offset_reg(i2c_dev, TLV320DAC3100_REG_DATA_SLOT_OFFSET);
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

  Adafruit_BusIO_Register offset_reg(i2c_dev, TLV320DAC3100_REG_DATA_SLOT_OFFSET);
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
  
  if (!n_en.write(enable)) return false;
  if (!n_val.write(val % 128)) return false;  // 0 represents 128

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
 * @brief Get the DAC and output driver status flags
 * @param left_dac_powered Pointer to store Left DAC power status, or NULL
 * @param hpl_powered Pointer to store HPL driver power status, or NULL
 * @param left_classd_powered Pointer to store Left Class-D power status, or NULL
 * @param right_dac_powered Pointer to store Right DAC power status, or NULL
 * @param hpr_powered Pointer to store HPR driver power status, or NULL
 * @param right_classd_powered Pointer to store Right Class-D power status, or NULL
 * @param left_pga_gain_ok Pointer to store Left PGA gain match status, or NULL
 * @param right_pga_gain_ok Pointer to store Right PGA gain match status, or NULL
 * @return true: success false: failure
 */
bool Adafruit_TLV320DAC3100::getDACFlags(bool *left_dac_powered, bool *hpl_powered, 
                                        bool *left_classd_powered, bool *right_dac_powered,
                                        bool *hpr_powered, bool *right_classd_powered,
                                        bool *left_pga_gain_ok, bool *right_pga_gain_ok) {
  if (!setPage(0)) {
    return false;
  }

  // Read first flag register
  Adafruit_BusIO_Register flag_reg(i2c_dev, TLV320DAC3100_REG_DAC_FLAG);
  Adafruit_BusIO_RegisterBits ldac_bit(&flag_reg, 1, 7);     // bit 7
  Adafruit_BusIO_RegisterBits hpl_bit(&flag_reg, 1, 5);      // bit 5
  Adafruit_BusIO_RegisterBits lclassd_bit(&flag_reg, 1, 4);  // bit 4
  Adafruit_BusIO_RegisterBits rdac_bit(&flag_reg, 1, 3);     // bit 3
  Adafruit_BusIO_RegisterBits hpr_bit(&flag_reg, 1, 1);      // bit 1
  Adafruit_BusIO_RegisterBits rclassd_bit(&flag_reg, 1, 0);  // bit 0

  // Read second flag register
  Adafruit_BusIO_Register flag2_reg(i2c_dev, TLV320DAC3100_REG_DAC_FLAG2);
  Adafruit_BusIO_RegisterBits lpga_bit(&flag2_reg, 1, 4);    // bit 4
  Adafruit_BusIO_RegisterBits rpga_bit(&flag2_reg, 1, 0);    // bit 0

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
