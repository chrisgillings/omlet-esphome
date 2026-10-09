#include "omlet_door.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome::omlet_door {

static const char *const TAG = "omlet_door";

static constexpr uint32_t DEAD_TIME_MS = 2;
static constexpr uint32_t SAMPLE_INTERVAL_MS = 10;
static constexpr uint32_t PUBLISH_INTERVAL_MS = 500;
static constexpr uint32_t CALIBRATION_PAUSE_MS = 1000;
static constexpr uint32_t MIN_SAMPLES_FOR_RATIO = 10;
static constexpr float FILTER_ALPHA = 0.3f;
static constexpr float EARLY_STALL_FRACTION = 0.7f;
static constexpr float TIME_OVERRUN = 1.15f;
static constexpr uint32_t CAL_PREF_SALT = 0x4F4D4C54;  // "OMLT"

using cover::COVER_CLOSED;
using cover::COVER_OPEN;
using cover::COVER_OPERATION_CLOSING;
using cover::COVER_OPERATION_IDLE;
using cover::COVER_OPERATION_OPENING;

void IRAM_ATTR CrushIsrStore::gpio_intr(CrushIsrStore *arg) {
  if (!arg->closing || !arg->crush.digital_read()) {
    return;
  }
  arg->high_side_a.digital_write(false);
  arg->high_side_b.digital_write(false);
  arg->low_side_a.digital_write(false);
  arg->low_side_b.digital_write(false);
  arg->tripped = true;
}

void OmletDoor::setup() {
  for (auto *pin : {this->high_side_a_, this->low_side_a_, this->high_side_b_, this->low_side_b_}) {
    pin->setup();
    pin->digital_write(false);
  }
  if (this->current_enable_pin_ != nullptr) {
    this->current_enable_pin_->setup();
    this->current_enable_pin_->digital_write(false);
  }

  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    this->position = COVER_CLOSED;
  }

  this->cal_pref_ =
      global_preferences->make_preference<CalibrationData>(this->get_object_id_hash() ^ CAL_PREF_SALT, true);
  CalibrationData cal{};
  if (this->cal_pref_.load(&cal)) {
    if (this->open_duration_ == 0) {
      this->open_duration_ = cal.open_ms;
    }
    if (this->close_duration_ == 0) {
      this->close_duration_ = cal.close_ms;
    }
    this->learned_threshold_ = cal.threshold;
  }

  if (this->crush_pin_ != nullptr) {
    this->crush_pin_->setup();
    this->isr_store_.crush = this->crush_pin_->to_isr();
    this->isr_store_.high_side_a = this->high_side_a_->to_isr();
    this->isr_store_.low_side_a = this->low_side_a_->to_isr();
    this->isr_store_.high_side_b = this->high_side_b_->to_isr();
    this->isr_store_.low_side_b = this->low_side_b_->to_isr();
    this->crush_pin_->attach_interrupt(&CrushIsrStore::gpio_intr, &this->isr_store_, gpio::INTERRUPT_ANY_EDGE);
  }

  this->apply_drive_(Drive::BRAKE);
  this->publish_state(false);
}

void OmletDoor::loop() {
  const uint32_t now = millis();

  if (this->drive_pending_ && now - this->drive_pending_since_ >= DEAD_TIME_MS) {
    this->drive_pending_ = false;
    if (!(this->isr_store_.closing && this->crush_active_())) {
      this->apply_drive_(this->pending_drive_);
    }
  }

  switch (this->state_) {
    case State::MOVING: {
      if (this->drive_pending_) {
        break;
      }
      const uint32_t elapsed = now - this->move_start_;

      if (now - this->last_sample_ >= SAMPLE_INTERVAL_MS) {
        this->last_sample_ = now;
        this->sample_current_();
      }

      if (this->move_op_ == COVER_OPERATION_CLOSING && (this->isr_store_.tripped || this->crush_active_())) {
        ESP_LOGW(TAG, "Crush switch triggered while closing%s", this->isr_store_.tripped ? " (cut by ISR)" : "");
        this->on_obstruction_();
        break;
      }

      if (elapsed >= this->start_sensing_delay_ && this->above_ && now - this->above_since_ >= this->stall_time_) {
        this->stall_level_ = this->filtered_current_;
        const bool early = this->move_op_ == COVER_OPERATION_CLOSING && this->purpose_ == Purpose::NORMAL &&
                           this->close_duration_ > 0 && elapsed < this->close_duration_ * EARLY_STALL_FRACTION;
        if (early) {
          ESP_LOGW(TAG, "High current %.3f V at %" PRIu32 " ms while closing, treating as obstruction",
                   this->filtered_current_, elapsed);
          this->on_obstruction_();
        } else {
          ESP_LOGD(TAG, "End stop reached at %" PRIu32 " ms, current %.3f V", elapsed, this->filtered_current_);
          this->on_end_stop_();
        }
        break;
      }

      if (this->update_position_(now)) {
        break;
      }

      const uint32_t duration =
          this->move_op_ == COVER_OPERATION_OPENING ? this->open_duration_ : this->close_duration_;
      if (this->purpose_ != Purpose::CALIBRATE_OPEN && this->purpose_ != Purpose::CALIBRATE_CLOSE && duration > 0 &&
          elapsed >= static_cast<uint32_t>(duration * TIME_OVERRUN)) {
        ESP_LOGD(TAG, "No stall seen, assuming end stop by time at %" PRIu32 " ms", elapsed);
        this->on_end_stop_();
        break;
      }

      if (elapsed >= this->max_duration_) {
        this->on_timeout_();
        break;
      }

      if (now - this->last_publish_ >= PUBLISH_INTERVAL_MS) {
        this->last_publish_ = now;
        this->publish_state(false);
      }
      break;
    }

    case State::RETRY_WAIT: {
      if (static_cast<int32_t>(now - this->retry_at_) < 0) {
        break;
      }
      if (this->crush_active_()) {
        ESP_LOGW(TAG, "Still crushed, delaying re-close");
        this->retry_at_ = now + this->crush_retry_delay_;
        break;
      }
      ESP_LOGI(TAG, "Re-closing after obstruction (attempt %u)", this->crush_count_);
      this->start_move_(COVER_OPERATION_CLOSING, Purpose::NORMAL);
      break;
    }

    default:
      break;
  }
}

void OmletDoor::dump_config() {
  LOG_COVER("", "Omlet Door", this);
  LOG_PIN("  High side A: ", this->high_side_a_);
  LOG_PIN("  Low side A: ", this->low_side_a_);
  LOG_PIN("  High side B: ", this->high_side_b_);
  LOG_PIN("  Low side B: ", this->low_side_b_);
  LOG_PIN("  Current enable pin: ", this->current_enable_pin_);
  LOG_PIN("  Crush pin: ", this->crush_pin_);
  ESP_LOGCONFIG(TAG,
                "  Invert direction: %s\n"
                "  Open duration: %" PRIu32 " ms\n"
                "  Close duration: %" PRIu32 " ms\n"
                "  Max duration: %" PRIu32 " ms\n"
                "  Start sensing delay: %" PRIu32 " ms\n"
                "  Stall time: %" PRIu32 " ms\n"
                "  Stall threshold: %.3f V (configured), %.3f V (learned), ratio %.2f\n"
                "  Max crush retries: %u, retry delay %" PRIu32 " ms",
                YESNO(this->invert_direction_), this->open_duration_, this->close_duration_, this->max_duration_,
                this->start_sensing_delay_, this->stall_time_, this->stall_threshold_, this->learned_threshold_,
                this->stall_ratio_, this->max_crush_retries_, this->crush_retry_delay_);
}

cover::CoverTraits OmletDoor::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_supports_toggle(true);
  traits.set_is_assumed_state(false);
  return traits;
}

void OmletDoor::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    if (this->state_ == State::MOVING) {
      this->stop_move_();
    }
    this->state_ = State::IDLE;
    this->purpose_ = Purpose::NORMAL;
    this->publish_state();
    return;
  }
  if (!call.get_position().has_value()) {
    return;
  }

  const float pos = *call.get_position();
  if (this->state_ == State::FAULT) {
    ESP_LOGI(TAG, "Clearing fault state on new command");
  }
  if (this->state_ == State::MOVING) {
    this->stop_move_();
  }
  this->state_ = State::IDLE;
  this->crush_count_ = 0;
  this->target_position_ = pos;

  if (pos != COVER_OPEN && pos != COVER_CLOSED && pos == this->position) {
    this->publish_state();
    return;
  }
  const auto op = pos > this->position || pos == COVER_OPEN ? COVER_OPERATION_OPENING : COVER_OPERATION_CLOSING;
  this->start_move_(op, Purpose::NORMAL);
}

void OmletDoor::calibrate() {
  if (this->state_ == State::MOVING) {
    this->stop_move_();
  }
  this->state_ = State::IDLE;
  this->crush_count_ = 0;
  ESP_LOGI(TAG, "Calibration started: closing until end stop");
  this->start_move_(COVER_OPERATION_CLOSING, Purpose::CALIBRATE_CLOSE);
}

void OmletDoor::start_move_(cover::CoverOperation op, Purpose purpose) {
  const uint32_t now = millis();
  this->move_op_ = op;
  this->purpose_ = purpose;
  this->state_ = State::MOVING;
  this->move_start_ = now;
  this->last_sample_ = now;
  this->last_position_update_ = now;
  this->last_publish_ = now;
  this->filter_primed_ = false;
  this->running_sum_ = 0.0f;
  this->running_count_ = 0;
  this->above_ = false;
  this->isr_store_.tripped = false;
  this->isr_store_.closing = op == COVER_OPERATION_CLOSING;
  this->current_operation = op;
  if (this->current_enable_pin_ != nullptr) {
    this->current_enable_pin_->digital_write(true);
  }
  const bool forward = (op == COVER_OPERATION_OPENING) != this->invert_direction_;
  this->set_drive_(forward ? Drive::FORWARD : Drive::REVERSE);
  this->high_freq_.start();
  this->publish_state(false);
}

void OmletDoor::stop_move_() {
  this->isr_store_.closing = false;
  this->set_drive_(Drive::BRAKE);
  this->state_ = State::IDLE;
  this->current_operation = COVER_OPERATION_IDLE;
  if (this->current_enable_pin_ != nullptr) {
    this->current_enable_pin_->digital_write(false);
  }
  this->high_freq_.stop();
}

void OmletDoor::set_drive_(Drive drive) {
  this->apply_drive_(Drive::OFF);
  this->pending_drive_ = drive;
  this->drive_pending_ = true;
  this->drive_pending_since_ = millis();
}

void OmletDoor::apply_drive_(Drive drive) {
  // Keep the four writes atomic with respect to the crush ISR, which clears all of them.
  InterruptLock lock;
  if (this->isr_store_.tripped && (drive == Drive::FORWARD || drive == Drive::REVERSE)) {
    drive = Drive::OFF;
  }
  this->high_side_a_->digital_write(drive == Drive::FORWARD);
  this->high_side_b_->digital_write(drive == Drive::REVERSE);
  this->low_side_a_->digital_write(drive == Drive::REVERSE || drive == Drive::BRAKE);
  this->low_side_b_->digital_write(drive == Drive::FORWARD || drive == Drive::BRAKE);
}

void OmletDoor::sample_current_() {
  const float v = this->current_sampler_->sample();
  if (std::isnan(v)) {
    return;
  }
  if (!this->filter_primed_) {
    this->filtered_current_ = v;
    this->filter_primed_ = true;
  } else {
    this->filtered_current_ += FILTER_ALPHA * (v - this->filtered_current_);
  }
  if (millis() - this->move_start_ < this->start_sensing_delay_) {
    return;
  }
  const float threshold = this->effective_threshold_();
  if (threshold > 0.0f && this->filtered_current_ > threshold) {
    if (!this->above_) {
      this->above_ = true;
      this->above_since_ = millis();
    }
    return;
  }
  this->above_ = false;
  this->running_sum_ += this->filtered_current_;
  this->running_count_++;
}

float OmletDoor::effective_threshold_() const {
  if (this->stall_threshold_ > 0.0f) {
    return this->stall_threshold_;
  }
  const bool calibrating = this->purpose_ == Purpose::CALIBRATE_OPEN || this->purpose_ == Purpose::CALIBRATE_CLOSE;
  if (!calibrating && this->learned_threshold_ > 0.0f) {
    return this->learned_threshold_;
  }
  if (this->running_count_ >= MIN_SAMPLES_FOR_RATIO) {
    return (this->running_sum_ / this->running_count_) * this->stall_ratio_;
  }
  return 0.0f;
}

bool OmletDoor::crush_active_() const { return this->crush_pin_ != nullptr && this->crush_pin_->digital_read(); }

void OmletDoor::on_end_stop_() {
  const uint32_t elapsed = millis() - this->move_start_;
  const float mean = this->running_count_ > 0 ? this->running_sum_ / this->running_count_ : 0.0f;
  const float candidate = mean + 0.5f * (this->stall_level_ - mean);
  this->stop_move_();
  this->position = this->move_op_ == COVER_OPERATION_OPENING ? COVER_OPEN : COVER_CLOSED;

  switch (this->purpose_) {
    case Purpose::CALIBRATE_CLOSE:
      this->close_duration_ = elapsed;
      this->cal_close_threshold_ = candidate;
      ESP_LOGI(TAG, "Calibration: close took %" PRIu32 " ms, running %.3f V, stall %.3f V", elapsed, mean,
               this->stall_level_);
      this->set_timeout(CALIBRATION_PAUSE_MS, [this]() {
        if (this->state_ == State::IDLE && this->purpose_ == Purpose::CALIBRATE_CLOSE) {
          this->start_move_(COVER_OPERATION_OPENING, Purpose::CALIBRATE_OPEN);
        }
      });
      break;
    case Purpose::CALIBRATE_OPEN:
      this->open_duration_ = elapsed;
      this->learned_threshold_ = 0.5f * (this->cal_close_threshold_ + candidate);
      this->purpose_ = Purpose::NORMAL;
      this->save_calibration_();
      ESP_LOGI(TAG, "Calibration complete: open %" PRIu32 " ms, close %" PRIu32 " ms, threshold %.3f V",
               this->open_duration_, this->close_duration_, this->learned_threshold_);
      break;
    case Purpose::CRUSH_REOPEN:
      this->purpose_ = Purpose::NORMAL;
      if (this->crush_count_ > this->max_crush_retries_) {
        this->enter_fault_("obstruction retries exhausted, door left open");
        return;
      }
      this->state_ = State::RETRY_WAIT;
      this->retry_at_ = millis() + this->crush_retry_delay_;
      break;
    case Purpose::NORMAL:
      if (this->move_op_ == COVER_OPERATION_CLOSING) {
        this->crush_count_ = 0;
      }
      break;
  }
  this->publish_state();
}

void OmletDoor::on_timeout_() {
  const bool calibrating = this->purpose_ == Purpose::CALIBRATE_OPEN || this->purpose_ == Purpose::CALIBRATE_CLOSE;
  this->stop_move_();
  if (calibrating) {
    this->purpose_ = Purpose::NORMAL;
    ESP_LOGE(TAG, "Calibration failed: no end stop detected within max_duration");
    this->publish_state();
    return;
  }
  this->enter_fault_("max_duration exceeded, position uncertain");
}

void OmletDoor::on_obstruction_() {
  this->crush_count_++;
  const bool calibrating = this->purpose_ == Purpose::CALIBRATE_OPEN || this->purpose_ == Purpose::CALIBRATE_CLOSE;
  if (calibrating) {
    ESP_LOGE(TAG, "Calibration aborted by obstruction");
  }
  this->stop_move_();
  ESP_LOGW(TAG, "Obstruction while closing (%u of %u), reopening", this->crush_count_, this->max_crush_retries_);
  this->start_move_(COVER_OPERATION_OPENING, Purpose::CRUSH_REOPEN);
}

bool OmletDoor::update_position_(uint32_t now) {
  const uint32_t duration = this->move_op_ == COVER_OPERATION_OPENING ? this->open_duration_ : this->close_duration_;
  const uint32_t dt = now - this->last_position_update_;
  this->last_position_update_ = now;
  if (duration == 0) {
    return false;
  }
  const float delta = static_cast<float>(dt) / static_cast<float>(duration);
  if (this->move_op_ == COVER_OPERATION_OPENING) {
    this->position = std::min(COVER_OPEN, this->position + delta);
  } else {
    this->position = std::max(COVER_CLOSED, this->position - delta);
  }
  if (this->purpose_ != Purpose::NORMAL || this->target_position_ == COVER_OPEN ||
      this->target_position_ == COVER_CLOSED) {
    return false;
  }
  const bool reached = this->move_op_ == COVER_OPERATION_OPENING ? this->position >= this->target_position_
                                                                 : this->position <= this->target_position_;
  if (!reached) {
    return false;
  }
  this->stop_move_();
  this->position = this->target_position_;
  this->publish_state();
  return true;
}

void OmletDoor::enter_fault_(const char *reason) {
  ESP_LOGE(TAG, "Fault: %s", reason);
  this->state_ = State::FAULT;
  this->purpose_ = Purpose::NORMAL;
  this->current_operation = COVER_OPERATION_IDLE;
  this->publish_state();
}

void OmletDoor::save_calibration_() {
  CalibrationData cal{this->open_duration_, this->close_duration_, this->learned_threshold_};
  this->cal_pref_.save(&cal);
}

}  // namespace esphome::omlet_door
