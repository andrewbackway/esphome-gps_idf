from esphome.components import uart
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
MULTI_CONF = True

CONF_GPS_IDF_ID = "gps_idf_id"

gps_idf_ns = cg.esphome_ns.namespace("gps_idf")
GPSIDFComponent = gps_idf_ns.class_(
    "GPSIDFComponent", cg.Component, uart.UARTDevice
)

GPS_IDF_CHILD_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_GPS_IDF_ID): cv.use_id(GPSIDFComponent),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(GPSIDFComponent),
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)