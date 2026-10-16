import esphome.codegen as cg
from esphome.components import text_sensor, uart, update
from esphome.components.http_request import CONF_HTTP_REQUEST_ID, HttpRequestComponent
from esphome.components.modbus_controller import ModbusController
import esphome.config_validation as cv
from esphome.const import CONF_UART_ID

from . import charger_update_ns

AUTO_LOAD = ["json"]
DEPENDENCIES = ["http_request", "modbus_controller", "rs485_mux", "uart"]

CONF_MUX_ID = "mux_id"
CONF_MODBUS_CONTROLLER_ID = "modbus_controller_id"
CONF_VERSION_SENSOR = "version_sensor"
CONF_REPO = "repo"

RS485Mux = cg.esphome_ns.namespace("rs485_mux").class_("RS485Mux", cg.Component)
ChargerUpdate = charger_update_ns.class_("ChargerUpdate", update.UpdateEntity, cg.PollingComponent)


def validate_repo(value):
    value = cv.string_strict(value)
    parts = value.split("/")
    if len(parts) != 2 or not all(parts) or any(c in value for c in " ?#"):
        raise cv.Invalid("repo must be OWNER/NAME, e.g. RAR/ec100-charger-fw")
    return value


CONFIG_SCHEMA = (
    update.update_schema(ChargerUpdate)
    .extend(
        {
            cv.GenerateID(CONF_HTTP_REQUEST_ID): cv.use_id(HttpRequestComponent),
            cv.Required(CONF_MUX_ID): cv.use_id(RS485Mux),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Required(CONF_MODBUS_CONTROLLER_ID): cv.use_id(ModbusController),
            cv.Required(CONF_VERSION_SENSOR): cv.use_id(text_sensor.TextSensor),
            cv.Optional(CONF_REPO, default="RAR/ec100-charger-fw"): validate_repo,
        }
    )
    .extend(cv.polling_component_schema("6h"))
)


async def to_code(config):
    var = await update.new_update(config)
    await cg.register_component(var, config)
    cg.add(var.set_http(await cg.get_variable(config[CONF_HTTP_REQUEST_ID])))
    cg.add(var.set_mux(await cg.get_variable(config[CONF_MUX_ID])))
    cg.add(var.set_uart(await cg.get_variable(config[CONF_UART_ID])))
    cg.add(var.set_controller(await cg.get_variable(config[CONF_MODBUS_CONTROLLER_ID])))
    cg.add(var.set_version_sensor(await cg.get_variable(config[CONF_VERSION_SENSOR])))
    cg.add(var.set_repo(config[CONF_REPO]))
