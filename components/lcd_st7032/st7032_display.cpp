#include "st7032_display.h"
#include "esphome/core/log.h"

namespace esphome::lcd_st7032 {

static const char *const TAG = "lcd_st7032";

// I2C control bytes: Co=0 plus RS bit.
static constexpr uint8_t CONTROL_COMMAND = 0x00;
static constexpr uint8_t CONTROL_DATA = 0x40;

// ST7032 instruction set, values taken from the Omlet stock firmware for this glass.
static constexpr uint8_t FUNCTION_SET_BASIC = 0x38;     // 8-bit, 2 lines, IS=0
static constexpr uint8_t FUNCTION_SET_EXTENDED = 0x39;  // 8-bit, 2 lines, IS=1
static constexpr uint8_t OSC_FREQUENCY = 0x13;          // bias 1/5, F=3
static constexpr uint8_t CONTRAST_LOW = 0x70;           // | C3..C0
static constexpr uint8_t POWER_ICON_CONTRAST = 0x54;    // icon off, booster on, | C5..C4
static constexpr uint8_t FOLLOWER_CONTROL = 0x68;       // follower on, Rab=0

void ST7032LCDDisplay::setup() {
  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->setup();
    this->enable_pin_->digital_write(true);
  }
  if (this->backlight_pin_ != nullptr) {
    this->backlight_pin_->setup();
    this->backlight_pin_->digital_write(true);
  }
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);
    delay(2);
    this->reset_pin_->digital_write(false);
    delay(2);
    this->reset_pin_->digital_write(true);
  }
  delay(40);

  if (this->write(nullptr, 0) != i2c::ERROR_OK) {
    this->mark_failed();
    return;
  }

  this->send(FUNCTION_SET_BASIC, false);
  this->send(FUNCTION_SET_EXTENDED, false);
  this->send(OSC_FREQUENCY, false);
  this->write_contrast_();
  this->send(FOLLOWER_CONTROL, false);
  delay(200);
  this->send(FUNCTION_SET_BASIC, false);

  LCDDisplay::setup();
}

void ST7032LCDDisplay::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ST7032 LCD Display:\n"
                "  Columns: %u, Rows: %u\n"
                "  Contrast: %u",
                this->columns_, this->rows_, this->contrast_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  LOG_PIN("  Backlight Pin: ", this->backlight_pin_);
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
  }
}

void ST7032LCDDisplay::send(uint8_t value, bool rs) {
  this->write_byte(rs ? CONTROL_DATA : CONTROL_COMMAND, value);
  delayMicroseconds(30);
}

void ST7032LCDDisplay::write_contrast_() {
  this->send(CONTRAST_LOW | (this->contrast_ & 0x0F), false);
  this->send(POWER_ICON_CONTRAST | ((this->contrast_ >> 4) & 0x03), false);
}

void ST7032LCDDisplay::set_contrast_live(uint8_t contrast) {
  this->contrast_ = contrast > 63 ? 63 : contrast;
  this->send(FUNCTION_SET_EXTENDED, false);
  this->write_contrast_();
  this->send(FUNCTION_SET_BASIC, false);
}

void ST7032LCDDisplay::backlight() {
  if (this->backlight_pin_ != nullptr) {
    this->backlight_pin_->digital_write(true);
  }
}

void ST7032LCDDisplay::no_backlight() {
  if (this->backlight_pin_ != nullptr) {
    this->backlight_pin_->digital_write(false);
  }
}

}  // namespace esphome::lcd_st7032
