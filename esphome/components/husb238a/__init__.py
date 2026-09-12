import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@sonocotta"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]

CONF_HUSB238A_ID = "husb238a_id"

husb238a_ns = cg.esphome_ns.namespace("husb238a")
Husb238aComponent = husb238a_ns.class_(
    "Husb238aComponent", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = cv.Schema(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Husb238aComponent),
        }
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(i2c.i2c_device_schema(0x42))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
