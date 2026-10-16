import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_UART_ID

DEPENDENCIES = ["uart"]

CONF_MODBUS_PORT = "modbus_port"
CONF_PASSTHROUGH_PORT = "passthrough_port"
CONF_IDLE_TIMEOUT = "idle_timeout"

rs485_mux_ns = cg.esphome_ns.namespace("rs485_mux")
RS485Mux = rs485_mux_ns.class_("RS485Mux", cg.Component)
# A UARTComponent subclass, so other components accept its ID as their uart_id.
MuxPort = rs485_mux_ns.class_("MuxPort", uart.UARTComponent)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RS485Mux),
            cv.Required(CONF_MODBUS_PORT): cv.Schema({cv.Required(CONF_ID): cv.declare_id(MuxPort)}),
            cv.Required(CONF_PASSTHROUGH_PORT): cv.Schema({cv.Required(CONF_ID): cv.declare_id(MuxPort)}),
            cv.Optional(CONF_IDLE_TIMEOUT, default="10min"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_uart(await cg.get_variable(config[CONF_UART_ID])))  # before set_ports: it syncs the settings
    modbus = cg.new_Pvariable(config[CONF_MODBUS_PORT][CONF_ID])
    stream = cg.new_Pvariable(config[CONF_PASSTHROUGH_PORT][CONF_ID])
    cg.add(var.set_ports(modbus, stream))
    cg.add(var.set_idle_timeout(config[CONF_IDLE_TIMEOUT]))
