import esphome.codegen as cg
from esphome.components import binary_sensor, sensor
from esphome.components.canbus import CanbusComponent
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_ID,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_REVOLUTIONS_PER_MINUTE,
    UNIT_VOLT,
    UNIT_WATT,
)

DEPENDENCIES = ["canbus"]
AUTO_LOAD = ["sensor", "binary_sensor"]
MULTI_CONF = True

CONF_CANBUS_ID = "canbus_id"
CONF_KEEPALIVE_INTERVAL = "keepalive_interval"
CONF_WRITE_STORED_DEFAULTS = "write_stored_defaults"
CONF_FALLBACK_VOLTAGE = "fallback_voltage"
CONF_FALLBACK_CURRENT = "fallback_current"
CONF_ALARM_MASK = "alarm_mask"
CONF_ONLINE = "online"
CONF_OUTPUT_POWER = "output_power"  # computed as output voltage x current (the NASN reports 0 in 0x0173)

nasn_rectifier_ns = cg.esphome_ns.namespace("nasn_rectifier")
NasnRectifier = nasn_rectifier_ns.class_("NasnRectifier", cg.PollingComponent)

# ValueKind in nasn_rectifier.h
TELEMETRY, VOLTS, AMPS, RAW = 0, 1, 2, 3


def _measure(unit, device_class, decimals):
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        device_class=device_class,
        accuracy_decimals=decimals,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _raw():
    return sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC)


def _rpm():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
        icon="mdi:fan",
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    )


# config key: (register, ValueKind, schema)
SENSORS = {
    "output_voltage": (0x0175, TELEMETRY, _measure(UNIT_VOLT, DEVICE_CLASS_VOLTAGE, 2)),
    # The stock firmware measures output voltage on 0x0184; which one is right on the NASN is unknown.
    "output_voltage_0184": (0x0184, TELEMETRY, _measure(UNIT_VOLT, DEVICE_CLASS_VOLTAGE, 2)),
    "output_current": (0x0182, TELEMETRY, _measure(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 2)),
    "input_voltage": (0x0178, TELEMETRY, _measure(UNIT_VOLT, DEVICE_CLASS_VOLTAGE, 1)),
    "input_frequency": (0x0171, TELEMETRY, _measure(UNIT_HERTZ, DEVICE_CLASS_FREQUENCY, 1)),
    "input_temperature": (0x0180, TELEMETRY, _measure(UNIT_CELSIUS, DEVICE_CLASS_TEMPERATURE, 1)),
    "output_temperature": (0x0235,  # NASN heatsink/outlet; 0x017F is not implemented
                            TELEMETRY, _measure(UNIT_CELSIUS, DEVICE_CLASS_TEMPERATURE, 1)),
    "voltage_setpoint": (0x0100, VOLTS, _measure(UNIT_VOLT, DEVICE_CLASS_VOLTAGE, 2)),
    "current_setpoint": (0x022D, AMPS, _measure(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 1)),
    "alarms": (0x0183, RAW, _raw()),
    "raw_021d": (0x021D, RAW, _raw()),
    "raw_022f": (0x022F, RAW, _raw()),
    "pfc_voltage": (0x0231, TELEMETRY, _measure(UNIT_VOLT, DEVICE_CLASS_VOLTAGE, 0)),
    "raw_0239": (0x0239, RAW, _raw()),
    "raw_023a": (0x023A, RAW, _raw()),
    "raw_023b": (0x023B, RAW, _raw()),
    "raw_024e": (0x024E, RAW, _raw()),
    # Fan tachometers (rpm, two fans per module): 0 with the fans stopped in standby, multiples of 150.
    "fan1_speed": (0x0242, RAW, _rpm()),
    "fan2_speed": (0x0243, RAW, _rpm()),
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NasnRectifier),
            cv.Required(CONF_CANBUS_ID): cv.use_id(CanbusComponent),
            cv.Required(CONF_ADDRESS): cv.int_range(min=1, max=127),
            cv.Optional(CONF_KEEPALIVE_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_WRITE_STORED_DEFAULTS, default=False): cv.boolean,
            # Kept conservative: the module runs on these by itself when CAN goes quiet
            # (FALLBACK_MAX_V / FALLBACK_MAX_A in nasn_rectifier.h).
            cv.Optional(CONF_FALLBACK_VOLTAGE, default=54.0): cv.float_range(min=42.0, max=54.4),
            cv.Optional(CONF_FALLBACK_CURRENT, default=5.0): cv.float_range(min=0.0, max=10.0),
            # 0x0183 data[4..7] bits the stock firmware stops on (E1-E11, nasn::STOCK_ALARM_MASK).
            cv.Optional(CONF_ALARM_MASK, default=0x0852501F): cv.hex_uint32_t,
            cv.Optional(CONF_ONLINE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            **{cv.Optional(key): schema for key, (_, _, schema) in SENSORS.items()},
            cv.Optional(CONF_OUTPUT_POWER): _measure(UNIT_WATT, DEVICE_CLASS_POWER, 0),
        }
    )
    .extend(cv.polling_component_schema("2s"))
)


async def to_code(config):
    canbus = await cg.get_variable(config[CONF_CANBUS_ID])
    var = cg.new_Pvariable(config[CONF_ID], canbus)
    await cg.register_component(var, config)
    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(var.set_keepalive_interval(config[CONF_KEEPALIVE_INTERVAL]))
    cg.add(var.set_write_stored_defaults(config[CONF_WRITE_STORED_DEFAULTS]))
    cg.add(var.set_alarm_mask(config[CONF_ALARM_MASK]))
    cg.add(var.set_fallback(config[CONF_FALLBACK_VOLTAGE], config[CONF_FALLBACK_CURRENT]))
    for key, (reg, kind, _) in SENSORS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(var.add_sensor(reg, kind, sens))
    if CONF_OUTPUT_POWER in config:
        power = await sensor.new_sensor(config[CONF_OUTPUT_POWER])
        cg.add(var.set_output_power_sensor(power))
    if CONF_ONLINE in config:
        online = await binary_sensor.new_binary_sensor(config[CONF_ONLINE])
        cg.add(var.set_online_binary_sensor(online))
