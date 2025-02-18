#include <Adafruit_TLV320DAC3100.h>

Adafruit_TLV320DAC3100 tlv;

void halt(const char *error_message) {
  Serial.println(error_message);
  while (1) yield();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("initialize codec");

  if (!tlv.begin()) {
    halt("Failed to initialize TLV320DAC3100!");
  }
  
  // Interface Control
  if (!tlv.setCodecInterface(TLV320DAC3100_FORMAT_I2S,
                            TLV320DAC3100_DATA_LEN_16)) {
    halt("Failed to configure codec interface!");
  }
  
  // Configure PLL and Clock System
  // 1. Set BCLK as PLL input and PLL as CODEC clock
  // 2. Configure PLL dividers (P=1, R=2, J=32, D=0)
  // 3. Set up DAC clock dividers (NDAC=8, MDAC=2)
  // 4. Finally power up the PLL
  if (!tlv.setPLLClockInput(TLV320DAC3100_PLL_CLKIN_BCLK) ||
      !tlv.setCodecClockInput(TLV320DAC3100_CODEC_CLKIN_PLL) ||
      !tlv.setPLLValues(1, 2, 32, 0) ||
      !tlv.setNDAC(true, 8) ||
      !tlv.setMDAC(true, 2) ||
      !tlv.powerPLL(true)) {
    halt("Failed to configure PLL and clock system!");
  }
  
  // Headset and GPIO Config
  if (!tlv.setHeadsetDetect(true)) {  // Enable with default debounce settings
    halt("Failed to enable headset detection!");
  }
  
  if (!tlv.setInt1Source(true, false, false, false, false, false)) {
    halt("Failed to configure INT1!");
  }
  
  if (!tlv.setGPIO1Mode(TLV320_GPIO1_INT1)) {  // Set as INT1 output
    halt("Failed to configure GPIO1!");
  }

  // DAC Setup
  if (!tlv.setDACDataPath(true, true)) {  // Enable both DACs with default paths
    halt("Failed to configure DAC!");
  }

  // DAC Routing
  if (!tlv.configureAnalogInputs(TLV320_DAC_ROUTE_MIXER,
                                TLV320_DAC_ROUTE_MIXER)) {
    halt("Failed to configure DAC routing!");
  }

  // DAC Volume Control
  if (!tlv.setDACVolumeControl(false, false, TLV320_VOL_INDEPENDENT)) {
    halt("Failed to set DAC volume control!");
  }

  if (!tlv.setChannelVolume(false, 3)) {  // Left channel, 3dB
    halt("Failed to set left DAC volume!");
  }
  
  if (!tlv.setChannelVolume(true, 3)) {  // Right channel, 3dB
    halt("Failed to set right DAC volume!");
  }
  
  // Configure Headphone System
  // 1. Enable headphone driver for both channels
  // 2. Set up left and right channel volumes (0x0A) and PGA gains (8dB)
  if (!tlv.configureHeadphoneDriver(true, true) ||
      !tlv.setHPLVolume(false, 0) ||    // 0 dB gain (max)
      !tlv.configureHPL_PGA(8, true) ||
      !tlv.setHPRVolume(false, 0) ||    // 0 dB gain (max)
      !tlv.configureHPR_PGA(8, true)) {
    halt("Failed to configure headphone system!");
  }

  
  // Configure Speaker System
  // 1. Enable speaker amplifier
  // 2. Configure PGA with 6dB gain
  // 3. Set speaker volume level
  if (!tlv.enableSpeaker(true) ||
      !tlv.configureSPK_PGA(TLV320_SPK_GAIN_6DB, true) ||
      !tlv.setSPKVolume(false, 0x0A)) {
    halt("Failed to configure speaker system!");
  }

  Serial.println("Initialization complete!");  
}

void loop() {
}
