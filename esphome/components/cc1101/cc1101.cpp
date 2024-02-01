#include "cc1101.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"

namespace esphome {
namespace cc1101 {

// CC1100 Misc
static const uint8_t CFG_REGISTER = 0x2F;       // 47 registers
static const uint8_t FIFOBUFFER = 0x42;         // size of Fifo Buffer
static const uint8_t RSSI_OFFSET_868MHZ = 0x4E; // dec = 74
static const uint8_t TX_RETRIES_MAX = 0x05;     // tx_retries_max
static const uint8_t ACK_TIMEOUT = 200;         // ACK timeout in ms
static const uint8_t CC1100_COMPARE_REGISTER =
    0x00; // register compare 0=no compare 1=compare
static const uint8_t BROADCAST_ADDRESS = 0x00; // broadcast address

// CC1100 R/W Offsets
static const uint8_t WRITE_SINGLE_BYTE = 0x00;
static const uint8_t WRITE_BURST = 0x40;
static const uint8_t READ_SINGLE_BYTE = 0x80;
static const uint8_t READ_BURST = 0xC0;

// CC1100 Commands
static const uint8_t TXFIFO_BURST = 0x7F;        // write burst only
static const uint8_t TXFIFO_SINGLE_BYTE = 0x3F;  // write single only
static const uint8_t RXFIFO_BURST = 0xFF;        // read burst only
static const uint8_t RXFIFO_SINGLE_BYTE = 0xBF;  // read single only
static const uint8_t PATABLE_BURST = 0x7E;       // power control read/write
static const uint8_t PATABLE_SINGLE_BYTE = 0xFE; // power control read/write

// CC1100 Config Register
static const uint8_t IOCFG2 = 0x00;   // GDO2 output pin configuration
static const uint8_t IOCFG1 = 0x01;   // GDO1 output pin configuration
static const uint8_t IOCFG0 = 0x02;   // GDO0 output pin configuration
static const uint8_t FIFOTHR = 0x03;  // RX FIFO and TX FIFO thresholds
static const uint8_t SYNC1 = 0x04;    // Sync word, high byte
static const uint8_t SYNC0 = 0x05;    // Sync word, low byte
static const uint8_t PKTLEN = 0x06;   // Packet length
static const uint8_t PKTCTRL1 = 0x07; // Packet automation control
static const uint8_t PKTCTRL0 = 0x08; // Packet automation control
static const uint8_t ADDR = 0x09;     // Device address
static const uint8_t CHANNR = 0x0A;   // Channel number
static const uint8_t FSCTRL1 = 0x0B;  // Frequency synthesizer control
static const uint8_t FSCTRL0 = 0x0C;  // Frequency synthesizer control
static const uint8_t FREQ2 = 0x0D;    // Frequency control word, high byte
static const uint8_t FREQ1 = 0x0E;    // Frequency control word, middle byte
static const uint8_t FREQ0 = 0x0F;    // Frequency control word, low byte
static const uint8_t MDMCFG4 = 0x10;  // Modem configuration
static const uint8_t MDMCFG3 = 0x11;  // Modem configuration
static const uint8_t MDMCFG2 = 0x12;  // Modem configuration
static const uint8_t MDMCFG1 = 0x13;  // Modem configuration
static const uint8_t MDMCFG0 = 0x14;  // Modem configuration
static const uint8_t DEVIATN = 0x15;  // Modem deviation setting
static const uint8_t MCSM2 = 0x16;    // Main Radio Cntrl State Machine config
static const uint8_t MCSM1 = 0x17;    // Main Radio Cntrl State Machine config
static const uint8_t MCSM0 = 0x18;    // Main Radio Cntrl State Machine config
static const uint8_t FOCCFG = 0x19;   // Frequency Offset Compensation config
static const uint8_t BSCFG = 0x1A;    // Bit Synchronization configuration
static const uint8_t AGCCTRL2 = 0x1B; // AGC control
static const uint8_t AGCCTRL1 = 0x1C; // AGC control
static const uint8_t AGCCTRL0 = 0x1D; // AGC control
static const uint8_t WOREVT1 = 0x1E;  // High byte Event 0 timeout
static const uint8_t WOREVT0 = 0x1F;  // Low byte Event 0 timeout
static const uint8_t WORCTRL = 0x20;  // Wake On Radio control
static const uint8_t FREND1 = 0x21;   // Front end RX configuration
static const uint8_t FREND0 = 0x22;   // Front end TX configuration
static const uint8_t FSCAL3 = 0x23;   // Frequency synthesizer calibration
static const uint8_t FSCAL2 = 0x24;   // Frequency synthesizer calibration
static const uint8_t FSCAL1 = 0x25;   // Frequency synthesizer calibration
static const uint8_t FSCAL0 = 0x26;   // Frequency synthesizer calibration
static const uint8_t RCCTRL1 = 0x27;  // RC oscillator configuration
static const uint8_t RCCTRL0 = 0x28;  // RC oscillator configuration
static const uint8_t FSTEST = 0x29;   // Frequency synthesizer cal control
static const uint8_t PTEST = 0x2A;    // Production test
static const uint8_t AGCTEST = 0x2B;  // AGC test
static const uint8_t TEST2 = 0x2C;    // Various test settings
static const uint8_t TEST1 = 0x2D;    // Various test settings
static const uint8_t TEST0 = 0x2E;    // Various test settings

// CC1100 Status Register
static const uint8_t PARTNUM = 0xF0;    // Part number
static const uint8_t VERSION = 0xF1;    // Current version number
static const uint8_t FREQEST = 0xF2;    // Frequency offset estimate
static const uint8_t LQI = 0xF3;        // Demodulator estimate for link quality
static const uint8_t RSSI = 0xF4;       // Received signal strength indication
static const uint8_t MARCSTATE = 0xF5;  // Control state machine state
static const uint8_t WORTIME1 = 0xF6;   // High byte of WOR timer
static const uint8_t WORTIME0 = 0xF7;   // Low byte of WOR timer
static const uint8_t PKTSTATUS = 0xF8;  // Current GDOx status and packet status
static const uint8_t VCO_VC_DAC = 0xF9; // Current setting from PLL cal module
static const uint8_t TXBYTES = 0xFA;    // Underflow and # of bytes in TXFIFO
static const uint8_t RXBYTES = 0xFB;    // Overflow and # of bytes in RXFIFO
static const uint8_t RCCTRL1_STATUS =
    0xFC; // Last RC Oscillator Calibration Result
static const uint8_t RCCTRL0_STATUS =
    0xFD; // Last RC Oscillator Calibration Result

// CC1100 Strobes
static const uint8_t SRES = 0x30;    // Reset chip
static const uint8_t SFSTXON = 0x31; // Enable/calibrate freq synthesizer
static const uint8_t SXOFF = 0x32;   // Turn off crystal oscillator.
static const uint8_t SCAL = 0x33;    // Calibrate freq synthesizer & disable
static const uint8_t SRX = 0x34;     // Enable RX.
static const uint8_t STX = 0x35;     // Enable TX.
static const uint8_t SIDLE = 0x36;   // Exit RX / TX
static const uint8_t SAFC = 0x37;    // AFC adjustment of freq synthesizer
static const uint8_t SWOR = 0x38;    // Start automatic RX polling sequence
static const uint8_t SPWD = 0x39;    // Enter pwr down mode when CSn goes hi
static const uint8_t SFRX = 0x3A;    // Flush the RX FIFO buffer.
static const uint8_t SFTX = 0x3B;    // Flush the TX FIFO buffer.
static const uint8_t SWORRST = 0x3C; // Reset real time clock.
static const uint8_t SNOP = 0x3D;    // No operation.

static const char *const TAG = "cc1101";

void CC1101Component::setup() {
  this->spi_setup();
  // check pin config
  // set ESP pin modes
  // SPI begin ?

  this->reset();

  this->strobe(SFTX); // Flush TX FIFO
  delayMicroseconds(100);

  this->strobe(SFRX); // Flush RX FIFO
  delayMicroseconds(100);

  this->part_number_ = this->read_reg(PARTNUM);
  this->chip_version_ = this->read_reg(VERSION);

  if (this->chip_version_ == 0x00 || this->chip_version_ == 0xFF) {
    ESP_LOGE(TAG, "Communication with the CC1101 failed");
    this->mark_failed();
    return;
  }

  // Set CC1101 config
  // Set CC1101 pin modes

  // Self test:
  //  - If GDO pins connected to ESP: verify connection
  //  - Detect the CC1101 crystal frequency (26MHz or 27MHz)

  if (do_gpio_self_test_) {
    this->write_reg(IOCFG2, 0x2E); // High impedance
    this->write_reg(IOCFG0, 0x2E); // High impedance

    if (this->gdo0_pin_) {
      this->gdo0_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
      this->pin_mode(0, gpio::FLAG_OUTPUT);
      gdo0_pin_connected_ = true;
      for (uint8_t i = 0; i < 3; i++) {
        this->digital_write(0, false);
        delayMicroseconds(100);
        if (this->gdo0_pin_->digital_read()) {
          gdo0_pin_connected_ = false;
          break;
        }
        this->digital_write(0, true);
        delayMicroseconds(100);
        if (!this->gdo0_pin_->digital_read()) {
          gdo0_pin_connected_ = false;
          break;
        }
      }
      this->gdo0_pin_->pin_mode(gpio::FLAG_INPUT);
      this->pin_mode(0, gpio::FLAG_NONE);
    }

    if (this->gdo2_pin_) {
      this->gdo2_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
      this->pin_mode(2, gpio::FLAG_OUTPUT);
      gdo2_pin_connected_ = true;
      for (uint8_t i = 0; i < 3; i++) {
        this->digital_write(2, false);
        delayMicroseconds(100);
        if (this->gdo2_pin_->digital_read()) {
          gdo2_pin_connected_ = false;
          break;
        }
        this->digital_write(2, true);
        delayMicroseconds(100);
        if (!this->gdo2_pin_->digital_read()) {
          gdo2_pin_connected_ = false;
          break;
        }
      }
      this->gdo2_pin_->pin_mode(gpio::FLAG_INPUT);
      this->pin_mode(2, gpio::FLAG_NONE);
    }

    if (this->gdo0_adc_) {
      // float voltage = this->gdo0_adc_->sample();
      // TODO: Connection self test
      // If not connected:
      //   this->gdo0_analog_pin_ = nullptr;
      this->pin_mode(0, gpio::FLAG_OUTPUT);
      gdo0_adc_connected_ = true;

      this->digital_write(0, false);
      delayMicroseconds(100);
      this->voltage_low_ = this->gdo0_adc_->sample();
      this->digital_write(0, true);
      delayMicroseconds(100);
      this->voltage_high_ = this->gdo0_adc_->sample();
      if (!std::isfinite(this->voltage_low_) || this->voltage_low_ > 0.6 ||
          this->voltage_high_ < 1.0)
        gdo0_adc_connected_ = false;
      // Pin is not connected or the ADC doesn't support the required voltage
      // range

      this->pin_mode(0, gpio::FLAG_NONE);
    }

    this->temperature_ = this->get_temperature(1);
  } else {
    // Skip gpio self test
    gdo0_pin_connected_ = (this->gdo0_pin_ != nullptr);
    gdo2_pin_connected_ = (this->gdo2_pin_ != nullptr);
    gdo0_adc_connected_ = (this->gdo0_adc_ != nullptr);
  }

  // this->marc_state_ = this->read_reg(MARCSTATE);
}

void CC1101Component::dump_config() {
  ESP_LOGCONFIG(TAG, "CC1101:");
  LOG_PIN("  GDO0 Pin: ", this->gdo0_pin_);
  // LOG_PIN("  GDO0 ADC: ", this->gdo0_analog_pin_);
  LOG_PIN("  GDO2 Pin: ", this->gdo2_pin_);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with CC1101 failed!");
  } else {
    ESP_LOGCONFIG(TAG, "  Chip version: %02x, Part number: %02x",
                  this->chip_version_, this->part_number_);
    ESP_LOGCONFIG(TAG,
                  "  GDO0 connected: %i, GDO2 connected: %i, ADC connected: %i",
                  this->gdo0_pin_connected_, this->gdo2_pin_connected_,
                  this->gdo0_adc_connected_);
    ESP_LOGCONFIG(
        TAG,
        "  ADC low voltage: %0.2f, high voltage: %0.2f, temperature: %0.2f",
        this->voltage_low_, this->voltage_high_, this->temperature_);
  }
}

void CC1101Component::loop() {
  // Shutdown CC1101
  //  this->reset_pin_->digital_write(false);
  // End transaction, disable CS pin
  //  this->disable();
  // Disable SPI HW
  //  this->disable_hardware();
  // Transfer byte
  //  this->transfer_byte(a);
  // Enable SPI HW
  //  this->enable_hardware();
  // Begin transaction, set SPI settings, enable CS pin
  //  this->enable();
}

void CC1101Component::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint8_t data;
  if (flags == gpio::FLAG_NONE || flags == gpio::FLAG_INPUT) {
    data = 0x2e; // High impedance
  } else if (flags == gpio::FLAG_OUTPUT) {
    data = 0x2f; // Low
  } else {
    return;
  }
  if (pin == 0) {
    // if (data == 0x2f && gdo0_last_output_level_)
    //   data = 0x6f; // High
    this->write_reg(IOCFG0, data);
  } else if (pin == 2) {
    // if (data == 0x2f && gdo2_last_output_level_)
    //   data = 0x6f; // High
    this->write_reg(IOCFG2, data);
  }
}

void CC1101Component::digital_write(uint8_t pin, bool value) {
  uint8_t data, old_data;
  if (value) {
    old_data = 0x2f; // Low
    data = 0x6f;     // High
  } else {
    old_data = 0x6f; // High
    data = 0x2f;     // Low
  }
  if (pin == 0) {
    // this->gdo0_last_output_level_ = value;
    if (this->iocfg0_ != old_data)
      return; // value is already correct or pin is not in output mode
    this->write_reg(IOCFG0, data);
  } else if (pin == 2) {
    // this->gdo2_last_output_level_ = value;
    if (this->iocfg2_ != old_data)
      return; // value is already correct or pin is not in output mode
    this->write_reg(IOCFG2, data);
  }
}

void CC1101Component::set_mode(CC1101Mode mode) {
  uint16_t timeout = 5000;
  uint8_t strobe_command;
  uint8_t target_marcstate;

  switch (mode) {
  case CC1101_MODE_IDLE:
  default:
    strobe_command = SIDLE;
    target_marcstate = 0x01; // IDLE
    break;
  case CC1101_MODE_TX:
    this->set_mode(CC1101_MODE_IDLE);
    strobe_command = STX;
    target_marcstate = 0x01; // IDLE after TX
    break;
  case CC1101_MODE_RX:
    this->set_mode(CC1101_MODE_IDLE);
    strobe_command = SRX;
    target_marcstate = 0x0d; // RX
    break;
  case CC1101_MODE_POWERDOWN:
    this->set_mode(CC1101_MODE_IDLE);
    this->strobe(SPWD);
    return;
  }
  this->strobe(strobe_command);
  while (((this->read_reg(MARCSTATE) & 0x1f) != target_marcstate) && timeout) {
    timeout--;
    delayMicroseconds(1);
  }
  delayMicroseconds(100);
}

void CC1101Component::reset() {
  this->enable();
  delayMicroseconds(10);
  this->disable();
  delayMicroseconds(40);
  strobe(SRES);
  delay(1);
}

void CC1101Component::wakeup() {
  this->enable();
  delayMicroseconds(10);
  this->disable();
  delayMicroseconds(10);
  this->set_mode(CC1101_MODE_IDLE); // CC1101_MODE_RX
}

void CC1101Component::set_wor(bool enable) {
  this->set_mode(CC1101_MODE_IDLE);
  if (enable) {
    this->write_reg(MCSM0, 0x18);   // FS autocalibration
    this->write_reg(MCSM2, 0x01);   // MCSM2.RX_TIME = 1b
    this->write_reg(WOREVT1, 0xff); // Event 0 timeout high byte
    this->write_reg(WOREVT0, 0x7f); // Event 0 timeout low byte
    this->write_reg(
        WORCTRL,
        0x78); // WOR_RES=0b; tEVENT1=0111b=48d -> 48*(750/26MHz)= 1.385ms
    this->strobe(SFRX);    // Flush RX buffer
    this->strobe(SWORRST); // resets the WOR timer to the programmed Event 1
    this->strobe(SWOR);    // put the radio in WOR mode when CSn is released
    delayMicroseconds(100);
  } else {
    this->write_reg(MCSM2, 0x07); // Stay in RX, no RX timeout
  }
}

void CC1101Component::reset_wor_timer() {
  this->set_mode(CC1101_MODE_IDLE);
  this->write_reg(MCSM2, 0x01); // MCSM2.RX_TIME = 1b
  this->strobe(SFRX);
  this->strobe(SWORRST);
  this->strobe(SWOR);
  delayMicroseconds(100);
}

void CC1101Component::set_channel(uint8_t channel) {
  this->write_reg(CHANNR, channel);
}

void CC1101Component::set_output_power(CC1101TxPower power) {
  this->write_reg(FREND0, power);
}

void CC1101Component::set_band(CC1101Band band) {
  std::vector<uint8_t> patable;
  switch (band) {
  case CC1101_BAND_315_MHZ:
    break;
  case CC1101_BAND_433_MHZ:
  default:
    break;
  case CC1101_BAND_868_MHZ:
    break;
  case CC1101_BAND_915_MHZ:
    break;
  }
  this->write_reg(FREQ2, 0);
  this->write_reg(FREQ1, 0);
  this->write_reg(FREQ0, 0);
  this->write_burst(PATABLE_BURST, patable);
}

/*void CC1101Component::set_modulation_preset(CC1101ModulationPreset preset) {
  std::vector<uint8_t> data;
  switch (preset) {
    case CC1101_PRESET_GFSK_1_2_kb:
      break;
  }
  this->write_burst(WRITE_BURST, data);
}*/

void CC1101Component::set_modulation(CC1101Modulation modulation) {
  uint8_t data;
  data = this->read_reg(MDMCFG2);
  data = (data & 0x8f) | (((modulation) << 4) & 0x70);
  this->write_reg(MDMCFG2, data);
}

float CC1101Component::get_temperature(uint8_t num_samples) {
  if (!this->gdo0_adc_) {
    return NAN;
  }
  if (num_samples == 0) {
    num_samples = 1;
  }
  if (this->gdo0_pin_ && gdo0_pin_connected_)
    this->gdo0_pin_->pin_mode(gpio::FLAG_INPUT);
  this->set_mode(CC1101_MODE_IDLE);
  uint8_t prev_iocfg0 = this->iocfg0_;
  this->write_reg(IOCFG0, 0x80); // Analog temperature output on GDO0
  this->write_reg(PTEST, 0xbf);  // Enable temperature sensor
  delay(50);

  float voltage = 0.0f;
  int successful_samples = 0;

  for (uint8_t i = 0; i < num_samples; i++) {
    float voltage_reading = this->gdo0_adc_->sample();
    if (std::isfinite(voltage_reading) && voltage_reading > 0.6 &&
        voltage_reading < 1.0) {
      voltage += voltage_reading;
      successful_samples++;
    }
  }

  this->write_reg(PTEST, 0x7f);         // Disable temperature sensor
  this->write_reg(IOCFG0, prev_iocfg0); // Restore GDO0 pin mode

  if (successful_samples)
    voltage /= static_cast<float>(successful_samples);
  else
    voltage = NAN;
  // TODO: calculate temperature

  // TODO: set CC1101 mode back to previous
  // this->set_mode(CC1101_MODE_RX);
  this->set_mode(CC1101_MODE_IDLE);
  return voltage; // TODO: return temperature
}

// SPI
// +++++++++++++++++++++++++++++++++++++++++++++++++++

void CC1101Component::strobe(uint8_t address) {
  if (this->is_failed()) {
    return;
  }
  this->enable();
  // TODO: Wait until MOSI pin is low
  this->write_byte(address);
  this->disable();
}

uint8_t CC1101Component::read_reg(uint8_t address) {
  if (this->is_failed()) {
    return 0;
  }
  this->enable();
  // TODO: Wait until MOSI pin is low
  this->write_byte(address | READ_SINGLE_BYTE);
  uint8_t data = this->transfer_byte(0xff);
  this->disable();
  return data;
}

std::vector<uint8_t> CC1101Component::read_burst(uint8_t address,
                                                 uint8_t length) {
  if (this->is_failed()) {
    return {};
  }
  this->enable();
  // TODO: Wait until MOSI pin is low
  this->write_byte(address | READ_BURST);
  std::vector<uint8_t> data;
  data.reserve(length);
  for (uint8_t i = 0; i < length; i++) {
    data.push_back(this->transfer_byte(0xff));
  }
  this->disable();
  return data;
}

void CC1101Component::write_reg(uint8_t address, uint8_t data) {
  if (this->is_failed()) {
    return;
  }
  this->enable();
  // TODO: Wait until MOSI pin is low
  this->write_byte(address | WRITE_SINGLE_BYTE);
  this->write_byte(data);
  this->disable();
  if (address == IOCFG0)
    this->iocfg0_ = data;
  else if (address == IOCFG2)
    this->iocfg2_ = data;
}

void CC1101Component::write_burst(uint8_t address, std::vector<uint8_t> data) {
  if (this->is_failed()) {
    return;
  }
  this->enable();
  // TODO: Wait until MOSI pin is low
  this->write_byte(address | WRITE_BURST);
  for (const uint8_t &d : data) {

    this->write_byte(d);
  }
  this->disable();
}

void CC1101GPIOPin::setup() { pin_mode(flags_); }
void CC1101GPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(this->pin_, flags);
}
bool CC1101GPIOPin::digital_read() {
  return false;
} // Not supported by hardware
void CC1101GPIOPin::digital_write(bool value) {
  this->parent_->digital_write(this->pin_, value != this->inverted_);
}
std::string CC1101GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via CC1101", pin_);
  return buffer;
}

} // namespace cc1101
} // namespace esphome

