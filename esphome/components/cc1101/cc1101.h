#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
//#include "esphome/core/automation.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"

namespace esphome {
namespace cc1101 {

static const uint32_t SPI_FREQUENCY = 6500000;

enum CC1101Modulation : uint8_t {
  CC1101_MODULATION_2_FSK = 0x00,
  CC1101_MODULATION_GFSK = 0x01,
  CC1101_MODULATION_ASK = 0x03,
  CC1101_MODULATION_4_FSK = 0x04,
  CC1101_MODULATION_MSK = 0x07,
};

enum CC1101TxPower : uint8_t {
  CC1101_TX_POWER_MINUS_30_DBM = 0x00,
  CC1101_TX_POWER_MINUS_20_DBM = 0x01,
  CC1101_TX_POWER_MINUS_15_DBM = 0x02,
  CC1101_TX_POWER_MINUS_10_DBM = 0x03,
  CC1101_TX_POWER_0_DBM = 0x04,
  CC1101_TX_POWER_5_DBM = 0x05,
  CC1101_TX_POWER_7_DBM = 0x06,
  CC1101_TX_POWER_10_DBM = 0x07,
};

enum CC1101Band {
  CC1101_BAND_315_MHZ,
  CC1101_BAND_433_MHZ,
  CC1101_BAND_868_MHZ,
  CC1101_BAND_915_MHZ,
};

enum CC1101Mode {
  CC1101_MODE_POWERDOWN,
  CC1101_MODE_IDLE,
  CC1101_MODE_RX,
  CC1101_MODE_TX,
};

class CC1101Component
    : public Component,
      public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                            spi::CLOCK_PHASE_LEADING,
                            spi::SPIDataRate(SPI_FREQUENCY)> {
public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  // float get_setup_priority() const override { return
  // esphome::setup_priority::DATA - 1.0f; }
  float get_setup_priority() const override {
    return esphome::setup_priority::AFTER_WIFI;
  }
  void set_gdo0_pin(GPIOPin *pin) { this->gdo0_pin_ = pin; }
  void set_gdo0_adc(voltage_sampler::VoltageSampler *sensor) {
    this->gdo0_adc_ = sensor;
  }
  void set_gdo2_pin(GPIOPin *pin) { this->gdo2_pin_ = pin; }
  void pin_mode(uint8_t pin, gpio::Flags flags);
  void digital_write(uint8_t pin, bool value);

  void set_mode(CC1101Mode mode);
  void reset();
  void wakeup();
  void set_wor(bool enable);
  void reset_wor_timer();
  void set_channel(uint8_t channel);
  void set_output_power(CC1101TxPower power);
  void set_band(CC1101Band band);
  void set_modulation(CC1101Modulation modulation);
  float get_temperature(uint8_t num_samples = 8);

protected:
  void strobe(uint8_t address);
  uint8_t read_reg(uint8_t address);
  std::vector<uint8_t> read_burst(uint8_t address, uint8_t length);
  void write_reg(uint8_t address, uint8_t data);
  void write_burst(uint8_t address, std::vector<uint8_t> data);

  GPIOPin *gdo0_pin_{nullptr};
  GPIOPin *gdo2_pin_{nullptr};
  voltage_sampler::VoltageSampler *gdo0_adc_{nullptr};
  bool do_gpio_self_test_{true};
  bool gdo0_pin_connected_{false};
  bool gdo2_pin_connected_{false};
  bool gdo0_adc_connected_{false};
  bool connection_error_{false};
  // bool gdo0_last_output_level_{false};
  // bool gdo2_last_output_level_{false};
  uint8_t iocfg0_{0x3f};
  uint8_t iocfg2_{0x29};
  uint8_t part_number_{0};
  uint8_t chip_version_{0};
  float voltage_low_{-1.0f};
  float voltage_high_{-1.0f};
  float temperature_{-1.0f};
};

class CC1101GPIOPin : public GPIOPin {
public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  void set_parent(CC1101Component *parent) { parent_ = parent; }
  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_inverted(bool inverted) { inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { flags_ = flags; }

protected:
  CC1101Component *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

} // namespace cc1101
} // namespace esphome

