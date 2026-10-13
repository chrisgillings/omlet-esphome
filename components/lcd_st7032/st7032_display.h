#pragma once

#include "esphome/components/display/display.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/lcd_base/lcd_display.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome::lcd_st7032 {

class ST7032LCDDisplay;

using st7032_lcd_writer_t = display::DisplayWriter<ST7032LCDDisplay>;

class ST7032LCDDisplay final : public lcd_base::LCDDisplay, public i2c::I2CDevice {
 public:
  void set_writer(st7032_lcd_writer_t &&writer) { this->writer_ = std::move(writer); }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }
  void set_backlight_pin(GPIOPin *pin) { this->backlight_pin_ = pin; }
  void set_contrast(uint8_t contrast) { this->contrast_ = contrast; }

  void setup() override;
  void dump_config() override;
  void backlight();
  void no_backlight();
  void set_contrast_live(uint8_t contrast);

 protected:
  bool is_four_bit_mode() override { return false; }
  void write_n_bits(uint8_t value, uint8_t n) override { this->send(value, false); }
  void send(uint8_t value, bool rs) override;
  void call_writer() override { this->writer_(*this); }
  void write_contrast_();

  GPIOPin *reset_pin_{nullptr};
  GPIOPin *enable_pin_{nullptr};
  GPIOPin *backlight_pin_{nullptr};
  uint8_t contrast_{22};
  st7032_lcd_writer_t writer_;
};

}  // namespace esphome::lcd_st7032
