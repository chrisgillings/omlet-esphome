#pragma once

#include "esphome/components/cover/cover.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

namespace esphome::omlet_door {

enum class Drive : uint8_t { OFF, FORWARD, REVERSE, BRAKE };

enum class State : uint8_t { IDLE, MOVING, RETRY_WAIT, FAULT };

enum class Purpose : uint8_t { NORMAL, CALIBRATE_PRE_OPEN, CALIBRATE_CLOSE, CALIBRATE_OPEN, OBSTRUCTION_RETURN };

struct CrushIsrStore {
  static void gpio_intr(CrushIsrStore *arg);

  ISRInternalGPIOPin crush;
  ISRInternalGPIOPin high_side_a;
  ISRInternalGPIOPin low_side_a;
  ISRInternalGPIOPin high_side_b;
  ISRInternalGPIOPin low_side_b;
  volatile bool closing{false};
  volatile bool tripped{false};
};

struct CalibrationData {
  uint32_t open_ms;
  uint32_t close_ms;
  float threshold;
  float running;
} __attribute__((packed));

class OmletDoor : public cover::Cover, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  cover::CoverTraits get_traits() override;

  void set_high_side_a(InternalGPIOPin *pin) { this->high_side_a_ = pin; }
  void set_low_side_a(InternalGPIOPin *pin) { this->low_side_a_ = pin; }
  void set_high_side_b(InternalGPIOPin *pin) { this->high_side_b_ = pin; }
  void set_low_side_b(InternalGPIOPin *pin) { this->low_side_b_ = pin; }
  void set_current_sampler(voltage_sampler::VoltageSampler *sampler) { this->current_sampler_ = sampler; }
  void set_current_enable_pin(GPIOPin *pin) { this->current_enable_pin_ = pin; }
  void set_crush_pin(InternalGPIOPin *pin) { this->crush_pin_ = pin; }
  void set_invert_direction(bool invert) { this->invert_direction_ = invert; }
  void set_open_duration(uint32_t ms) { this->open_duration_ = ms; }
  void set_close_duration(uint32_t ms) { this->close_duration_ = ms; }
  void set_max_duration(uint32_t ms) { this->max_duration_ = ms; }
  void set_start_sensing_delay(uint32_t ms) { this->start_sensing_delay_ = ms; }
  void set_stall_time(uint32_t ms) { this->stall_time_ = ms; }
  void set_stall_threshold(float v) { this->stall_threshold_ = v; }
  void set_stall_ratio(float r) { this->stall_ratio_ = r; }
  void set_min_current(float v) { this->min_current_ = v; }
  void set_max_crush_retries(uint8_t n) { this->max_crush_retries_ = n; }
  void set_crush_retry_delay(uint32_t ms) { this->crush_retry_delay_ = ms; }

  void calibrate();
  bool has_fault() const { return this->state_ == State::FAULT; }
  const char *get_fault_reason() const { return this->has_fault() ? this->fault_reason_ : ""; }

 protected:
  void control(const cover::CoverCall &call) override;

  void start_move_(cover::CoverOperation op, Purpose purpose);
  void stop_move_();
  void set_drive_(Drive drive);
  void apply_drive_(Drive drive);
  void sample_current_();
  float effective_threshold_() const;
  float effective_min_current_() const;
  bool crush_active_() const;
  bool is_calibrating_() const;
  uint32_t start_window_() const;
  void on_end_stop_();
  void on_timeout_();
  void on_obstruction_();
  void start_relief_();
  void cancel_relief_();
  bool update_position_(uint32_t now);
  void enter_fault_(const char *reason);
  void save_calibration_();

  InternalGPIOPin *high_side_a_{nullptr};
  InternalGPIOPin *low_side_a_{nullptr};
  InternalGPIOPin *high_side_b_{nullptr};
  InternalGPIOPin *low_side_b_{nullptr};
  voltage_sampler::VoltageSampler *current_sampler_{nullptr};
  GPIOPin *current_enable_pin_{nullptr};
  InternalGPIOPin *crush_pin_{nullptr};
  CrushIsrStore isr_store_;

  bool invert_direction_{false};
  uint32_t open_duration_{0};
  uint32_t close_duration_{0};
  uint32_t max_duration_{30000};
  uint32_t start_sensing_delay_{300};
  uint32_t stall_time_{100};
  float stall_threshold_{0.0f};
  float stall_ratio_{2.0f};
  float min_current_{0.0f};
  uint8_t max_crush_retries_{3};
  uint32_t crush_retry_delay_{10000};

  ESPPreferenceObject cal_pref_;
  float learned_threshold_{0.0f};
  float learned_running_{0.0f};

  State state_{State::IDLE};
  Purpose purpose_{Purpose::NORMAL};
  cover::CoverOperation move_op_{cover::COVER_OPERATION_IDLE};
  cover::CoverOperation retry_op_{cover::COVER_OPERATION_IDLE};
  float target_position_{0.0f};
  uint8_t crush_count_{0};

  Drive pending_drive_{Drive::OFF};
  Drive relief_drive_{Drive::OFF};
  bool drive_pending_{false};
  uint32_t drive_pending_since_{0};

  uint32_t move_start_{0};
  uint32_t last_sample_{0};
  uint32_t last_position_update_{0};
  uint32_t last_publish_{0};
  uint32_t retry_at_{0};

  float filtered_current_{0.0f};
  bool filter_primed_{false};
  float running_sum_{0.0f};
  uint32_t running_count_{0};
  float stall_level_{0.0f};
  float cal_close_threshold_{0.0f};
  const char *fault_reason_{""};
  uint32_t above_since_{0};
  bool above_{false};
  uint32_t below_since_{0};
  bool below_{false};

  HighFrequencyLoopRequester high_freq_;
};

template<typename... Ts> class CalibrateAction : public Action<Ts...> {
 public:
  explicit CalibrateAction(OmletDoor *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->calibrate(); }

 protected:
  OmletDoor *parent_;
};

}  // namespace esphome::omlet_door
