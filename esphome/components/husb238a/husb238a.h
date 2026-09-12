#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/preferences.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif

namespace esphome {
namespace husb238a {

// Register map reverse-engineered from https://github.com/Pythonic-Rainbow/HUSB238A and cross-checked
// against the Hynetek HUSB238A datasheet's Theory of Operation section. This is NOT the same layout as
// the plain HUSB238 (addresses 0x00-0x09) -- HUSB238A moves everything to a much larger map, and gates
// GO_COMMAND behind an ENABLE bit that the plain HUSB238 doesn't have.
enum class CommandRegister : uint8_t {
  CONTROL1 = 0x02,
  GO_COMMAND = 0x18,
  SRC_PDO = 0x19,
  STATUS = 0x63,
};

// STATUS(0x63) .. SRC_PDO_20V(0x6E), read as one contiguous block.
const uint8_t STATUS_BLOCK_NUM = 12;
const uint8_t STATUS_BLOCK_START = 0x63;

enum class GoCommandFunction : uint8_t {
  SELECT_PDO = 0b00001,
  GET_SRC_CAP = 0b00100,
  SOFT_RESET = 0b11101,
  HARD_RESET = 0b11110,
};

enum class PdoSelect : uint8_t {
  NOT_SELECTED = 0b00000,
  SRC_PDO_5V = 0b00001,
  SRC_PDO_9V = 0b00010,
  SRC_PDO_12V = 0b00011,
  SRC_PDO_15V = 0b00100,
  SRC_PDO_20V = 0b00101,
};

// Same encoding as PdoSelect, reported back by the chip once a contract is established.
enum class PdContract : uint8_t {
  TYPE_C_5V = 0b0000,
  SRC_PDO_5V = 0b0001,
  SRC_PDO_9V = 0b0010,
  SRC_PDO_12V = 0b0011,
  SRC_PDO_15V = 0b0100,
  SRC_PDO_20V = 0b0101,
};

union RegControl1 {
  struct {
    uint8_t tccdeb : 3;
    bool enable : 1;
    bool vdm_respond : 1;
    bool en_dpm_hiz : 1;
    uint8_t reserved : 2;
  };
  uint8_t raw;
};

union RegGoCommand {
  struct {
    uint8_t function : 5;
    uint8_t reserved : 3;
  };
  uint8_t raw;
};

union RegSrcPdoSelect {
  struct {
    uint8_t reserved : 3;
    uint8_t pdo_select : 5;
  };
  uint8_t raw;
};

union RegStatus {
  struct {
    bool attach : 1;
    uint8_t bc_lvl : 2;
    bool tsd : 1;
    uint8_t reserved : 2;
    bool pd_epr_snk : 1;
    bool ams_process : 1;
  };
  uint8_t raw;
};

union RegStatus1 {
  struct {
    bool data_role : 1;
    bool fault : 1;
    bool ams_succ : 1;
    bool src_alert : 1;
    bool pd_comm : 1;
    bool pd_hv : 1;
    bool reserved : 1;
    bool flgin : 1;
  };
  uint8_t raw;
};

union RegContractStatus0 {
  struct {
    uint8_t dpm_contract : 4;
    uint8_t pd_contract : 4;
  };
  uint8_t raw;
};

// CONTRACT_STATUS1 is a plain value register (see current_to_ma_() for the scale), not a bitfield.

union RegSrcPdoXxv {
  struct {
    uint8_t current : 7;  // 100 mA per LSB
    bool detect : 1;
  };
  uint8_t raw;
};

class Husb238aComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  Husb238aComponent() = default;

  void setup() override;
  void update() override;
  void dump_config() override;

  bool command_request_voltage(int volt);
  bool command_request_voltage(const std::string &select_state);
  bool command_request_pdo(PdoSelect voltage);

  bool is_attached();

  void defer_update() {
    this->set_timeout(DEFER_UPDATE_DELAY_MS, [this]() { this->update(); });
  }

#ifdef USE_SENSOR
  SUB_SENSOR(voltage)
  SUB_SENSOR(current)
  SUB_SENSOR(selected_voltage)
#endif

#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(status)
  SUB_TEXT_SENSOR(capabilities)
#endif

#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(attached)
#endif

#ifdef USE_SELECT
  SUB_SELECT(voltage)
#endif

 protected:
  static const uint32_t DEFER_UPDATE_DELAY_MS = 300;

  struct {
    RegStatus status;
    RegStatus1 status1;
    uint8_t type;
    uint8_t dpdm_status;
    RegContractStatus0 contract_status0;
    uint8_t contract_status1;
    uint8_t source_cap_info;
    RegSrcPdoXxv src_pdo_5v;
    RegSrcPdoXxv src_pdo_9v;
    RegSrcPdoXxv src_pdo_12v;
    RegSrcPdoXxv src_pdo_15v;
    RegSrcPdoXxv src_pdo_20v;
  } registers_;

  bool ready_{false};
  ESPPreferenceObject voltage_pref_;

  bool enable_chip_();
  void restore_requested_voltage_();
  bool read_status_block_(bool &is_changed);
  bool send_command_(GoCommandFunction function);
  bool select_pdo_(PdoSelect voltage);
  std::string get_capabilities_();
};

#ifdef USE_SELECT
class Husb238aVoltageSelect : public select::Select, public Parented<Husb238aComponent> {
 public:
  Husb238aVoltageSelect() = default;

 protected:
  void control(const std::string &value) override {
    this->publish_state(value);
    this->parent_->command_request_voltage(value);
    this->parent_->defer_update();
  };
};
#endif

}  // namespace husb238a
}  // namespace esphome
