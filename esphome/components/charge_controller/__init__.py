import esphome.codegen as cg
from esphome.components import binary_sensor, number, sensor, switch, text_sensor
from esphome.components.nasn_rectifier import NasnRectifier
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_PROBLEM,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_SECOND,
    UNIT_VOLT,
    UNIT_WATT,
)

DEPENDENCIES = ["mqtt", "nasn_rectifier"]
AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor"]

CONF_RECTIFIERS = "rectifiers"
CONF_MAX_VOLTAGE = "max_voltage"
CONF_MAX_CURRENT = "max_current"
CONF_FALLBACK_VOLTAGE = "fallback_voltage"
CONF_FALLBACK_CURRENT = "fallback_current"
CONF_PAUSE = "pause"
CONF_TOPICS = "topics"
CONF_NO_CURRENT = "no_current"
CONF_CHARGING_INSTRUCTION = "charging_instruction"
DEFAULT_CHARGING_INSTRUCTION = "yambms/yambms_1/sensor/charging_instruction/state"

charge_controller_ns = cg.esphome_ns.namespace("charge_controller")
ChargeController = charge_controller_ns.class_("ChargeController", cg.Component)

# Order matches charge::Topic in charge_logic.h.
TOPICS = {
    "status": "yambms/status",
    "charge_voltage": "yambms/yambms_1/sensor/requested_charge_voltage/state",
    "charge_current": "yambms/yambms_1/sensor/requested_charge_current/state",
    "charging_allowed": "yambms/yambms_1/binary_sensor/charging_allowed/state",
    "alarm": "yambms/yambms_1/sensor/bms_in_alarm/state",
    "current": "yambms/yambms_1/sensor/current/state",
    "blocking": "yambms/yambms_1/sensor/bms_blocking_charge/state",
    "max_cell": "yambms/yambms_1/sensor/max_cell_voltage/state",
    "battery_voltage": "yambms/yambms_1/sensor/total_voltage/state",
}

NUMBERS = {
    CONF_MAX_VOLTAGE: "set_max_voltage_number",
    CONF_MAX_CURRENT: "set_max_current_number",
    CONF_FALLBACK_VOLTAGE: "set_fallback_voltage_number",
    CONF_FALLBACK_CURRENT: "set_fallback_current_number",
}
# Optional: unset falls back to charge::CELL_LIMIT_V / CELL_RESUME_V.
OPTIONAL_NUMBERS = {
    "max_cell_voltage": "set_max_cell_voltage_number",
    "cell_resume_voltage": "set_cell_resume_voltage_number",
}


def _measure(unit, device_class, decimals):
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        device_class=device_class,
        accuracy_decimals=decimals,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _age():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_SECOND,
        device_class=DEVICE_CLASS_DURATION,
        accuracy_decimals=0,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )


SENSORS = {
    "target_voltage": ("set_target_voltage_sensor", _measure(UNIT_VOLT, DEVICE_CLASS_VOLTAGE, 2)),
    "target_current": ("set_target_current_sensor", _measure(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 1)),
    "budget": ("set_budget_sensor", _measure(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 1)),
    "output_power": ("set_output_power_sensor", _measure(UNIT_WATT, DEVICE_CLASS_POWER, 0)),
    "bms_age": ("set_bms_age_sensor", _age()),
    "max_cell_age": ("set_max_cell_age_sensor", _age()),
}
TEXT_SENSORS = {
    "state": "set_state_text_sensor",
    "last_fault": "set_last_fault_text_sensor",
    "hold_reason": "set_hold_reason_text_sensor",
    "bms_charge_stage": "set_bms_charge_stage_text_sensor",
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ChargeController),
        cv.Required(CONF_RECTIFIERS): cv.All(cv.ensure_list(cv.use_id(NasnRectifier)), cv.Length(min=1)),
        **{cv.Required(key): cv.use_id(number.Number) for key in NUMBERS},
        **{cv.Optional(key): cv.use_id(number.Number) for key in OPTIONAL_NUMBERS},
        cv.Required(CONF_PAUSE): cv.use_id(switch.Switch),
        cv.Optional(CONF_TOPICS, default={}): cv.Schema(
            {
                **{cv.Optional(key, default=topic): cv.subscribe_topic for key, topic in TOPICS.items()},
                cv.Optional(CONF_CHARGING_INSTRUCTION, default=DEFAULT_CHARGING_INSTRUCTION): cv.subscribe_topic,
            }
        ),
        **{cv.Optional(key): schema for key, (_, schema) in SENSORS.items()},
        **{cv.Optional(key): text_sensor.text_sensor_schema() for key in TEXT_SENSORS},
        cv.Optional(CONF_NO_CURRENT): binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_PROBLEM),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    for rid in config[CONF_RECTIFIERS]:
        cg.add(var.add_rectifier(await cg.get_variable(rid)))
    for key, setter in NUMBERS.items():
        cg.add(getattr(var, setter)(await cg.get_variable(config[key])))
    for key, setter in OPTIONAL_NUMBERS.items():
        if key in config:
            cg.add(getattr(var, setter)(await cg.get_variable(config[key])))
    cg.add(var.set_pause_switch(await cg.get_variable(config[CONF_PAUSE])))
    for index, key in enumerate(TOPICS):
        cg.add(var.set_topic(index, config[CONF_TOPICS][key]))
    cg.add(var.set_instruction_topic(config[CONF_TOPICS][CONF_CHARGING_INSTRUCTION]))
    for key, (setter, _) in SENSORS.items():
        if key in config:
            cg.add(getattr(var, setter)(await sensor.new_sensor(config[key])))
    for key, setter in TEXT_SENSORS.items():
        if key in config:
            cg.add(getattr(var, setter)(await text_sensor.new_text_sensor(config[key])))
    if CONF_NO_CURRENT in config:
        cg.add(var.set_no_current_binary_sensor(await binary_sensor.new_binary_sensor(config[CONF_NO_CURRENT])))
