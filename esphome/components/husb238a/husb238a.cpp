#include "husb238a.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cstring>
#include <map>

#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif
#ifdef USE_ESP32
#include <esp_system.h>
#include "soc/soc_caps.h"
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#include <driver/usb_serial_jtag.h>
#define HUSB238A_HAS_USJ 1
#endif
#endif

namespace esphome {
namespace husb238a {

static const char *const TAG = "husb238a";

static const std::map<std::string, PdoSelect> SELECT_VOLTAGE{
    {"5V", PdoSelect::SRC_PDO_5V},
    {"9V", PdoSelect::SRC_PDO_9V},
    {"12V", PdoSelect::SRC_PDO_12V},
    {"15V", PdoSelect::SRC_PDO_15V},
    {"20V", PdoSelect::SRC_PDO_20V},
};

static float pdo_select_to_voltage(uint8_t pdo_select) {
  switch (static_cast<PdoSelect>(pdo_select)) {
    case PdoSelect::SRC_PDO_5V:
      return 5.0f;
    case PdoSelect::SRC_PDO_9V:
      return 9.0f;
    case PdoSelect::SRC_PDO_12V:
      return 12.0f;
    case PdoSelect::SRC_PDO_15V:
      return 15.0f;
    case PdoSelect::SRC_PDO_20V:
      return 20.0f;
    default:
      return 0.0f;
  }
}

// Mirrors pdo_select_to_voltage(), but as the option string the select platform publishes/expects.
static const char *pdo_select_to_string(uint8_t pdo_select) {
  switch (static_cast<PdoSelect>(pdo_select)) {
    case PdoSelect::SRC_PDO_5V:
      return "5V";
    case PdoSelect::SRC_PDO_9V:
      return "9V";
    case PdoSelect::SRC_PDO_12V:
      return "12V";
    case PdoSelect::SRC_PDO_15V:
      return "15V";
    case PdoSelect::SRC_PDO_20V:
      return "20V";
    default:
      return nullptr;
  }
}

static float pd_contract_to_voltage(uint8_t pd_contract) {
  switch (static_cast<PdContract>(pd_contract)) {
    case PdContract::TYPE_C_5V:
      return 5.0f;
    case PdContract::SRC_PDO_5V:
      return 5.0f;
    case PdContract::SRC_PDO_9V:
      return 9.0f;
    case PdContract::SRC_PDO_12V:
      return 12.0f;
    case PdContract::SRC_PDO_15V:
      return 15.0f;
    case PdContract::SRC_PDO_20V:
      return 20.0f;
    default:
      return 0.0f;
  }
}

// CONTRACT_STATUS1 encodes the negotiated operating current for a fixed PDO contract: 20 mA/LSB up to
// 0x7D, 40 mA/LSB above that, offset by 500 mA. Source: Pythonic-Rainbow/HUSB238A Conversion.hpp
// (from_contract_current_fpdo), not documented in the public Hynetek datasheet.
static float contract_current_to_amps(uint8_t contract_status1) {
  int step = contract_status1 <= 0x7D ? 20 : 40;
  return (500 + contract_status1 * step) / 1000.0f;
}

// SRC_PDO_xV.current is 100 mA/LSB.
static float src_pdo_current_to_amps(uint8_t current_field) { return current_field * 0.1f; }

// Steps recorded by crumb_(). Kept small on purpose; they are printed at the next boot.
enum CrumbStep : uint8_t {
  CRUMB_NONE = 0,
  CRUMB_SETUP = 1,
  CRUMB_ALREADY_ENABLED = 2,
  CRUMB_HOST_SKIPPED = 3,
  CRUMB_ENABLED = 4,
  CRUMB_SOURCE_TALKING = 5,
  CRUMB_SOURCE_SILENT = 6,
  CRUMB_RESTORE_WAITING = 7,
  CRUMB_RESTORE_SAME = 8,
  CRUMB_RESTORE_NOT_OFFERED = 9,
  CRUMB_RESTORE_REQUESTED = 10,
  CRUMB_FIRST_UPDATE = 11,
  CRUMB_HEARTBEAT = 12,
};

static const char *crumb_name(uint8_t step) {
  switch (step) {
    case CRUMB_SETUP:
      return "setup";
    case CRUMB_ALREADY_ENABLED:
      return "already enabled (warm reboot)";
    case CRUMB_HOST_SKIPPED:
      return "USB host attached, PD skipped";
    case CRUMB_ENABLED:
      return "policy engine enabled";
    case CRUMB_SOURCE_TALKING:
      return "source answered, kept enabled";
    case CRUMB_SOURCE_SILENT:
      return "source silent, disabled again";
    case CRUMB_RESTORE_WAITING:
      return "waiting for start-up negotiation";
    case CRUMB_RESTORE_SAME:
      return "saved voltage already active / none saved";
    case CRUMB_RESTORE_NOT_OFFERED:
      return "saved voltage not offered";
    case CRUMB_RESTORE_REQUESTED:
      return "saved voltage requested";
    case CRUMB_FIRST_UPDATE:
      return "first status poll done";
    case CRUMB_HEARTBEAT:
      return "still running (heartbeat)";
    default:
      return "none";
  }
}

void Husb238aComponent::crumb_(uint8_t step) {
  if (!this->boot_trace_)
    return;
  Crumb c{};
  c.step = step;
  c.contract0 = this->registers_.contract_status0.raw;
  c.contract1 = this->registers_.contract_status1;
  c.caps = (this->registers_.src_pdo_5v.detect ? 0x01 : 0) | (this->registers_.src_pdo_9v.detect ? 0x02 : 0) |
           (this->registers_.src_pdo_12v.detect ? 0x04 : 0) | (this->registers_.src_pdo_15v.detect ? 0x08 : 0) |
           (this->registers_.src_pdo_20v.detect ? 0x10 : 0);
  c.ms = millis();
#ifdef USE_WIFI
  // Bit 7 of the caps mask: Wi-Fi connected at the time of the step.
  if (wifi::global_wifi_component != nullptr && wifi::global_wifi_component->is_connected())
    c.caps |= 0x80;
#endif
  if (step == CRUMB_HEARTBEAT) {
    // The heartbeat only ever occupies the last slot, so the start-up steps stay intact.
    if (this->crumb_log_.count < CRUMB_LOG_SIZE)
      this->crumb_log_.count = CRUMB_LOG_SIZE;
    this->crumb_log_.entries[CRUMB_LOG_SIZE - 1] = c;
  } else if (this->crumb_log_.count >= CRUMB_LOG_SIZE) {
    // Keep the first entries (the start-up sequence is what matters) and the newest one.
    this->crumb_log_.entries[CRUMB_LOG_SIZE - 1] = c;
  } else {
    this->crumb_log_.entries[this->crumb_log_.count++] = c;
  }
  this->crumb_pref_.save(&this->crumb_log_);
  global_preferences->sync();
}

void Husb238aComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HUSB238A...");

  // Key renamed on purpose: earlier builds wrote a 5 V default here that the user never chose, and
  // that value must not be restored now that an absent preference means "keep the chip's contract".
  this->voltage_pref_ = global_preferences->make_preference<uint8_t>(fnv1_hash("husb238a_user_voltage") ^
                                                                       this->get_i2c_address());
  this->crumb_pref_ =
      global_preferences->make_preference<CrumbLog>(fnv1_hash("husb238a_crumblog") ^ this->get_i2c_address());
  CrumbLog prev{};
  if (this->boot_trace_ && this->crumb_pref_.load(&prev) && prev.count > 0 && prev.count <= CRUMB_LOG_SIZE) {
    ESP_LOGI(TAG, "Previous run (it started from reset reason %u), PD steps in order (uptime ms, contract0/contract1, caps mask [bit7 = Wi-Fi up]):",
             prev.start_reason);
    for (uint8_t i = 0; i < prev.count; i++) {
      const Crumb &c = prev.entries[i];
      if (c.step == CRUMB_NONE)
        continue;
      ESP_LOGI(TAG, "  %6u ms  0x%02X/0x%02X  0x%02X  %s", (unsigned) c.ms, c.contract0, c.contract1, c.caps,
               crumb_name(c.step));
    }
  }
  this->crumb_log_ = CrumbLog{};
#ifdef USE_ESP32
  this->crumb_log_.start_reason = static_cast<uint8_t>(esp_reset_reason());
#endif
  this->crumb_(CRUMB_SETUP);
  // Heartbeat every 10 s: tells how long the previous run lived when it ended without a console.
  if (this->boot_trace_) {
    this->set_interval("husb238a_heartbeat", 10000, [this]() { this->crumb_(CRUMB_HEARTBEAT); });
  }

  // HUSB238A gates its whole PD policy engine behind CONTROL1.ENABLE: with the bit clear the chip
  // only answers I2C (datasheet, "Enable Control"), presents Rd and lets the board run on the
  // Type-C 5 V default. Register reads/writes are ACKed either way, so a missing ENABLE write is
  // silent: commands appear to succeed but never reach the policy engine.
  RegControl1 control1;
  if (!this->read_byte(static_cast<uint8_t>(CommandRegister::CONTROL1), &control1.raw)) {
    ESP_LOGE(TAG, "Failed to read CONTROL1");
    this->mark_failed();
    return;
  }

  if (control1.enable) {
    // Warm reboot (OTA, reset button, watchdog): VBUS never dropped, the chip kept running and its
    // explicit contract is still in place. Nothing to negotiate, just re-apply the saved voltage.
    ESP_LOGD(TAG, "Policy engine already enabled, keeping the current contract");
    this->crumb_(CRUMB_ALREADY_ENABLED);
    this->pd_state_ = PdState::RUNNING;
    this->ready_ = true;
    this->set_timeout(DEFER_UPDATE_DELAY_MS, [this]() {
      this->restore_requested_voltage_();
      this->defer_update();
    });
    return;
  }

  // Cold start of the policy engine. Two traps, both verified on a Louder-ESP32 Mini 55:
  //
  // 1. Setting ENABLE makes the chip attach as a PD sink and wait for Source_Capabilities. A
  //    source only advertises them for a few seconds after the plug goes in; if we enable later
  //    than that (after flashing, or on a reset while plugged in), the sink hears nothing and --
  //    as the PD spec requires -- issues a Hard Reset about 500 ms after attach. The source then
  //    removes VBUS for up to a second, the ESP32 brown-outs, and because the chip is powered from
  //    VBUS too it comes back with ENABLE clear, so a naive driver repeats this forever (FSM 0x02
  //    -> 0x2A, then "E BOD: Brownout detector was triggered"). So: enable, watch the chip at high
  //    rate, and if the source hasn't talked to us within PROBE_WINDOW_MS switch the engine off
  //    again before its Hard Reset timer expires. See loop()/finish_enable_().
  //
  // 2. When the board hangs off a computer, the PD contract itself kills the USB data link: macOS
  //    (and some other hosts) only keep their data port enabled if the sink's Request carries the
  //    "USB Communications Capable" bit, which this chip's factory configuration does not set. A
  //    computer port only supplies 5 V anyway, so when the ESP32's own USB port sees a host we
  //    simply don't start PD. See usb_host_attached_().
  this->pd_state_ = PdState::PROBING;
  this->set_timeout("husb238a_enable", HOST_CHECK_DELAY_MS, [this, control1]() {
    if (this->skip_with_usb_host_ && this->usb_host_attached_()) {
      this->pd_state_ = PdState::HOST_SKIPPED;
      this->crumb_(CRUMB_HOST_SKIPPED);
      ESP_LOGI(TAG,
               "USB host attached (board is plugged into a computer): not starting USB-PD, the host would drop the "
               "USB data link once a contract is negotiated. Staying on the Type-C 5 V default.");
#ifdef USE_TEXT_SENSOR
      if (this->status_text_sensor_ != nullptr) {
        this->status_text_sensor_->publish_state("Not negotiated (USB host attached)");
      }
      if (this->capabilities_text_sensor_ != nullptr) {
        this->capabilities_text_sensor_->publish_state("Unknown (USB-PD not started)");
      }
#endif
#ifdef USE_BINARY_SENSOR
      if (this->attached_binary_sensor_ != nullptr) {
        this->attached_binary_sensor_->publish_state(false);
      }
#endif
      return;
    }
    this->start_policy_engine_(control1);
  });
}

void Husb238aComponent::start_policy_engine_(RegControl1 control1) {
  control1.enable = true;
  if (!this->write_byte(static_cast<uint8_t>(CommandRegister::CONTROL1), control1.raw)) {
    ESP_LOGE(TAG, "Failed to enable HUSB238A");
    this->mark_failed();
    return;
  }
  this->enable_time_ = millis();
  this->high_freq_.start();
  this->crumb_(CRUMB_ENABLED);
}

// True while the ESP32-S3/C3/C6 USB Serial/JTAG port receives SOF packets, i.e. a USB host is on
// the other end of the cable. A charger or power bank never sends SOFs. Without that peripheral
// (or on other chips) we cannot tell and assume there is no host.
bool Husb238aComponent::usb_host_attached_() {
#ifdef HUSB238A_HAS_USJ
  return usb_serial_jtag_is_connected();
#else
  return false;
#endif
}

void Husb238aComponent::loop() {
  if (this->pd_state_ != PdState::PROBING || this->enable_time_ == 0) {
    return;
  }
  if (millis() - this->enable_time_ < PROBE_WINDOW_MS) {
    return;
  }
  this->high_freq_.stop();
  this->finish_enable_(this->source_is_talking_());
}

// True once the source has sent us anything: a capability list (SRC_PDO_xV.detect), an explicit
// contract, or just PD traffic (STATUS1.PD_COMM).
bool Husb238aComponent::source_is_talking_() {
  bool is_changed{false};
  if (!this->read_status_block_(is_changed)) {
    return false;
  }
  const RegSrcPdoXxv *pdos[] = {&this->registers_.src_pdo_5v, &this->registers_.src_pdo_9v,
                                 &this->registers_.src_pdo_12v, &this->registers_.src_pdo_15v,
                                 &this->registers_.src_pdo_20v};
  for (auto *pdo : pdos) {
    if (pdo->detect) {
      return true;
    }
  }
  return this->registers_.status1.pd_comm || this->registers_.contract_status0.pd_contract != 0;
}

void Husb238aComponent::finish_enable_(bool source_is_talking) {
  if (source_is_talking) {
    ESP_LOGI(TAG, "USB-PD source answered within %u ms, policy engine enabled", (unsigned) PROBE_WINDOW_MS);
    this->pd_state_ = PdState::RUNNING;
    this->ready_ = true;
    this->crumb_(CRUMB_SOURCE_TALKING);
    // The chip is negotiating its own first contract right now; wait for that AMS to settle
    // before asking for the saved voltage, then again before the first status poll.
    this->set_timeout(DEFER_UPDATE_DELAY_MS, [this]() {
      this->restore_requested_voltage_();
      this->defer_update();
    });
    return;
  }

  RegControl1 control1;
  if (this->read_byte(static_cast<uint8_t>(CommandRegister::CONTROL1), &control1.raw)) {
    control1.enable = false;
    this->write_byte(static_cast<uint8_t>(CommandRegister::CONTROL1), control1.raw);
  }
  this->pd_state_ = PdState::DISABLED;
  this->crumb_(CRUMB_SOURCE_SILENT);
  ESP_LOGW(TAG,
           "No USB-PD source capabilities within %u ms of enabling the chip: the source stopped advertising "
           "(board reset while plugged in, or a non-PD supply). Policy engine disabled again to avoid a PD Hard "
           "Reset that would cut VBUS and brown out the board. Staying on the Type-C 5 V default; unplug for 30 s "
           "and plug back in to negotiate USB-PD.",
           (unsigned) PROBE_WINDOW_MS);
  this->status_set_warning(LOG_STR("USB-PD disabled, power cycle needed"));

#ifdef USE_BINARY_SENSOR
  if (this->attached_binary_sensor_ != nullptr) {
    this->attached_binary_sensor_->publish_state(false);
  }
#endif
#ifdef USE_TEXT_SENSOR
  if (this->status_text_sensor_ != nullptr) {
    this->status_text_sensor_->publish_state("Disabled (power cycle needed)");
  }
  if (this->capabilities_text_sensor_ != nullptr) {
    this->capabilities_text_sensor_->publish_state("Unknown (USB-PD disabled)");
  }
#endif
}

bool Husb238aComponent::pdo_offered_(PdoSelect voltage) {
  switch (voltage) {
    case PdoSelect::SRC_PDO_5V:
      return this->registers_.src_pdo_5v.detect;
    case PdoSelect::SRC_PDO_9V:
      return this->registers_.src_pdo_9v.detect;
    case PdoSelect::SRC_PDO_12V:
      return this->registers_.src_pdo_12v.detect;
    case PdoSelect::SRC_PDO_15V:
      return this->registers_.src_pdo_15v.detect;
    case PdoSelect::SRC_PDO_20V:
      return this->registers_.src_pdo_20v.detect;
    default:
      return false;
  }
}

void Husb238aComponent::restore_requested_voltage_() {
  // On power-up the chip negotiates its own factory-configured contract (on the Sonocotta boards
  // that is the highest voltage the source offers, e.g. 20 V on a 60 W charger) and forgets
  // whatever fixed voltage was last requested over I2C. Re-request the saved voltage here so a
  // reboot doesn't silently change it. When nothing was ever saved, leave the chip's own contract
  // alone: forcing a default here would mean a needless VBUS step (20 V -> 5 V) while the rest of
  // the board is still starting up.
  uint8_t stored = 0;
  if (!this->voltage_pref_.load(&stored) || stored < static_cast<uint8_t>(PdoSelect::SRC_PDO_5V) ||
      stored > static_cast<uint8_t>(PdoSelect::SRC_PDO_20V)) {
    ESP_LOGD(TAG, "No saved voltage, keeping the contract the chip negotiated on its own");
    this->crumb_(CRUMB_RESTORE_SAME);
    return;
  }
  const auto want = static_cast<PdoSelect>(stored);
  const uint32_t started = millis();
  this->crumb_(CRUMB_RESTORE_WAITING);

  // GO_COMMAND is ignored while the chip is in the middle of an AMS (its own start-up
  // negotiation, typically). Poll until it is idle, then request -- but only a PDO the source
  // actually offers, and only if that isn't already the contract.
  this->set_interval("husb238a_restore", 100, [this, want, started]() {
    bool is_changed{false};
    if (!this->read_status_block_(is_changed)) {
      return;
    }
    if (this->registers_.status.ams_process && millis() - started < RESTORE_TIMEOUT_MS) {
      return;
    }
    this->cancel_interval("husb238a_restore");

    if (this->registers_.contract_status0.pd_contract == static_cast<uint8_t>(want)) {
      ESP_LOGD(TAG, "Saved voltage %.0f V is already the active contract", pdo_select_to_voltage(static_cast<uint8_t>(want)));
      this->crumb_(CRUMB_RESTORE_SAME);
      return;
    }
    if (!this->pdo_offered_(want)) {
      ESP_LOGW(TAG, "Saved voltage %.0f V is not offered by this source (%s), keeping the current contract",
               pdo_select_to_voltage(static_cast<uint8_t>(want)), this->get_capabilities_().c_str());
      this->crumb_(CRUMB_RESTORE_NOT_OFFERED);
      return;
    }
    this->crumb_(CRUMB_RESTORE_REQUESTED);
    if (!this->command_request_pdo(want)) {
      ESP_LOGW(TAG, "Failed to restore last requested PD voltage");
    }
    this->defer_update();
  });
}

// SRC_PDO holds the PDO last requested over I2C and reads 0 until that happens. Before any
// request the chip's own contract is what's "selected", so fall back to it.
uint8_t Husb238aComponent::effective_pdo_select_() {
  RegSrcPdoSelect src_pdo{};
  if (this->read_byte(static_cast<uint8_t>(CommandRegister::SRC_PDO), &src_pdo.raw) && src_pdo.pdo_select != 0) {
    return src_pdo.pdo_select;
  }
  return this->registers_.status.attach ? this->registers_.contract_status0.pd_contract : 0;
}

void Husb238aComponent::update() {
  if (!this->ready_) {
    return;
  }

  bool is_changed{false};
  if (!this->read_status_block_(is_changed)) {
    is_changed = !this->status_has_error();
    this->status_set_error(LOG_STR("Unable to communicate with HUSB238A chip"));
    memset(&this->registers_, 0, sizeof(this->registers_));
  } else {
    this->status_clear_error();
  }

  // The status block has usually been read already during start-up (probe window, voltage
  // restore), so the first poll sees "no change". Publish once unconditionally or the entities
  // stay empty until the contract changes.
  if (!this->published_once_) {
    this->published_once_ = true;
    is_changed = true;
    this->crumb_(CRUMB_FIRST_UPDATE);
  }

  if (!is_changed) {
    return;
  }

#ifdef USE_BINARY_SENSOR
  if (this->attached_binary_sensor_ != nullptr) {
    this->attached_binary_sensor_->publish_state(this->registers_.status.attach);
  }
#endif

#ifdef USE_SENSOR
  if (this->voltage_sensor_ != nullptr) {
    float voltage = this->registers_.status.attach
                        ? pd_contract_to_voltage(this->registers_.contract_status0.pd_contract)
                        : 0.0f;
    this->voltage_sensor_->publish_state(voltage);
  }

  if (this->current_sensor_ != nullptr) {
    float current =
        this->registers_.status.attach ? contract_current_to_amps(this->registers_.contract_status1) : 0.0f;
    this->current_sensor_->publish_state(current);
  }

  if (this->selected_voltage_sensor_ != nullptr) {
    this->selected_voltage_sensor_->publish_state(pdo_select_to_voltage(this->effective_pdo_select_()));
  }
#endif

#ifdef USE_SELECT
  if (this->voltage_select_ != nullptr) {
    // Keep the select in sync with what the chip actually has selected -- it otherwise never
    // gets an initial state (shows "Unknown") until the user changes it themselves.
    if (const char *option = pdo_select_to_string(this->effective_pdo_select_())) {
      this->voltage_select_->publish_state(option);
    }
  }
#endif

#ifdef USE_TEXT_SENSOR
  if (this->status_text_sensor_ != nullptr) {
    this->status_text_sensor_->publish_state(this->registers_.status1.ams_succ ? "Success" : "Unknown");
  }

  if (this->capabilities_text_sensor_ != nullptr) {
    this->capabilities_text_sensor_->publish_state(this->get_capabilities_());
  }
#endif
}

void Husb238aComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HUSB238A:");
  const char *state = "idle";
  switch (this->pd_state_) {
    case PdState::PROBING:
      state = "probing source";
      break;
    case PdState::RUNNING:
      state = "running";
      break;
    case PdState::DISABLED:
      state = "disabled (source silent at boot, power cycle needed)";
      break;
    case PdState::HOST_SKIPPED:
      state = "not started (USB host attached)";
      break;
    default:
      break;
  }
  ESP_LOGCONFIG(TAG, "  Policy engine: %s", state);
  ESP_LOGCONFIG(TAG, "  Skip PD when a USB host is attached: %s", YESNO(this->skip_with_usb_host_));
  ESP_LOGCONFIG(TAG, "  Boot trace in flash: %s", YESNO(this->boot_trace_));

#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "PD Attached", this->attached_binary_sensor_);
#endif

#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Source Voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "Source Current", this->current_sensor_);
  LOG_SENSOR("  ", "Selected Voltage", this->selected_voltage_sensor_);
#endif

#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Last Request Status", this->status_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Capabilities", this->capabilities_text_sensor_);
#endif

#ifdef USE_SELECT
  LOG_SELECT("  ", "Voltage", this->voltage_select_);
#endif
}

bool Husb238aComponent::command_request_voltage(int volt) {
  PdoSelect voltage;
  switch (volt) {
    case 5:
      voltage = PdoSelect::SRC_PDO_5V;
      break;
    case 9:
      voltage = PdoSelect::SRC_PDO_9V;
      break;
    case 12:
      voltage = PdoSelect::SRC_PDO_12V;
      break;
    case 15:
      voltage = PdoSelect::SRC_PDO_15V;
      break;
    case 20:
      voltage = PdoSelect::SRC_PDO_20V;
      break;
    default:
      ESP_LOGE(TAG, "Invalid voltage");
      return false;
  }
  return this->command_request_pdo(voltage);
}

bool Husb238aComponent::command_request_voltage(const std::string &select_state) {
  auto volt = SELECT_VOLTAGE.find(select_state);
  if (volt == SELECT_VOLTAGE.end()) {
    ESP_LOGE(TAG, "Invalid voltage");
    return false;
  }
  return this->command_request_pdo(volt->second);
}

bool Husb238aComponent::command_request_pdo(PdoSelect voltage) {
  if (!this->ready_) {
    if (this->pd_state_ == PdState::DISABLED) {
      ESP_LOGE(TAG, "USB-PD is disabled (source was silent at boot); power-cycle the board to negotiate");
    } else if (this->pd_state_ == PdState::HOST_SKIPPED) {
      ESP_LOGE(TAG, "USB-PD not started because a USB host is attached; use a charger for USB-PD");
    } else {
      ESP_LOGE(TAG, "Component not ready");
    }
    return false;
  }
  if (!this->pdo_offered_(voltage)) {
    ESP_LOGW(TAG, "%.0f V is not offered by this source (%s), not requesting it",
             pdo_select_to_voltage(static_cast<uint8_t>(voltage)), this->get_capabilities_().c_str());
    return false;
  }

  if (!this->select_pdo_(voltage)) {
    ESP_LOGV(TAG, "Select PDO voltage failed");
    return false;
  }
  delay(5);

  if (!this->send_command_(GoCommandFunction::SELECT_PDO)) {
    ESP_LOGV(TAG, "Send SELECT_PDO failed");
    return false;
  }

  uint8_t stored = static_cast<uint8_t>(voltage);
  this->voltage_pref_.save(&stored);

  return true;
}

bool Husb238aComponent::is_attached() {
  if (!this->ready_) {
    return false;
  }
  return this->registers_.status.attach;
}

bool Husb238aComponent::read_status_block_(bool &is_changed) {
  uint16_t old_crc = crc16(reinterpret_cast<uint8_t *>(&this->registers_), sizeof(this->registers_));

  auto ok = this->read_bytes(STATUS_BLOCK_START, reinterpret_cast<uint8_t *>(&this->registers_), STATUS_BLOCK_NUM);
  if (!ok) {
    ESP_LOGE(TAG, "Error reading HUSB238A status block");
  }
  is_changed = old_crc != crc16(reinterpret_cast<uint8_t *>(&this->registers_), sizeof(this->registers_));

  return ok;
}

bool Husb238aComponent::send_command_(GoCommandFunction function) {
  ESP_LOGV(TAG, "Sending command %u", static_cast<uint8_t>(function));

  if (!this->ready_) {
    ESP_LOGE(TAG, "Component not ready");
    return false;
  }
  RegGoCommand go_command{};
  go_command.function = static_cast<uint8_t>(function);
  auto ok = this->write_byte(static_cast<uint8_t>(CommandRegister::GO_COMMAND), go_command.raw);
  if (!ok) {
    ESP_LOGE(TAG, "Error sending command to HUSB238A");
  }
  return ok;
}

bool Husb238aComponent::select_pdo_(PdoSelect voltage) {
  ESP_LOGV(TAG, "Setting PDO select to %.0f", pdo_select_to_voltage(static_cast<uint8_t>(voltage)));
  if (!this->ready_) {
    ESP_LOGE(TAG, "Component not ready");
    return false;
  }

  RegSrcPdoSelect reg_data{};
  reg_data.pdo_select = static_cast<uint8_t>(voltage);
  auto ok = this->write_byte(static_cast<uint8_t>(CommandRegister::SRC_PDO), reg_data.raw);
  if (!ok) {
    ESP_LOGE(TAG, "Error setting PDO select");
  }
  return ok;
}

std::string Husb238aComponent::get_capabilities_() {
  const RegSrcPdoXxv *pdos[] = {&this->registers_.src_pdo_5v, &this->registers_.src_pdo_9v,
                                 &this->registers_.src_pdo_12v, &this->registers_.src_pdo_15v,
                                 &this->registers_.src_pdo_20v};
  const uint8_t voltages[] = {5, 9, 12, 15, 20};

  bool nothing_detected = true;
  for (auto *pdo : pdos) {
    if (pdo->detect) {
      nothing_detected = false;
      break;
    }
  }
  if (nothing_detected) {
    return "No capabilities detected";
  }

  std::string capabilities;
  capabilities.reserve(96);
  char buffer[32];
  for (size_t i = 0; i < 5; i++) {
    if (pdos[i]->detect) {
      snprintf(buffer, sizeof(buffer), "%dV: %.1fA", voltages[i], src_pdo_current_to_amps(pdos[i]->current));
      if (!capabilities.empty()) {
        capabilities += ", ";
      }
      capabilities += buffer;
    }
  }
  return capabilities;
}

}  // namespace husb238a
}  // namespace esphome
