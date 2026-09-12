import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv
from esphome.const import CONF_NAME, CONF_VOLTAGE, ENTITY_CATEGORY_CONFIG

from . import CONF_HUSB238A_ID, Husb238aComponent, husb238a_ns

CODEOWNERS = ["@sonocotta"]
DEPENDENCIES = ["husb238a"]

ICON_KNOB = "mdi:knob"

Husb238aVoltageSelect = husb238a_ns.class_("Husb238aVoltageSelect", select.Select)

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_HUSB238A_ID): cv.use_id(Husb238aComponent),
    cv.Optional(CONF_VOLTAGE): cv.maybe_simple_value(
        select.select_schema(
            Husb238aVoltageSelect,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_KNOB,
        ),
        key=CONF_NAME,
    ),
}


async def to_code(config):
    hub = await cg.get_variable(config[CONF_HUSB238A_ID])

    if voltage_selector := config.get(CONF_VOLTAGE):
        s = await select.new_select(
            voltage_selector,
            options=["5V", "9V", "12V", "15V", "20V"],
        )
        await cg.register_parented(s, config[CONF_HUSB238A_ID])
        cg.add(hub.set_voltage_select(s))
