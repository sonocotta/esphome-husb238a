import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import CONF_NAME, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_HUSB238A_ID, Husb238aComponent

CODEOWNERS = ["@sonocotta"]
DEPENDENCIES = ["husb238a"]

CONF_ATTACHED = "attached"

TYPES = [CONF_ATTACHED]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_HUSB238A_ID): cv.use_id(Husb238aComponent),
            cv.Optional(CONF_ATTACHED): cv.maybe_simple_value(
                binary_sensor.binary_sensor_schema(
                    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                ),
                key=CONF_NAME,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


async def setup_conf(config, key, hub):
    if sensor_config := config.get(key):
        var = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(getattr(hub, f"set_{key}_binary_sensor")(var))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_HUSB238A_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
