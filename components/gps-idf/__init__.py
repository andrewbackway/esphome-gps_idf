# components/gps_idf/__init__.py
from esphome import automation, config_validation as cv
import esphome.codegen as cg
from esphome.components import uart, sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    UNIT_DEGREES,
    UNIT_METER,
    UNIT_KILOMETER_PER_HOUR,
)

AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor", "uart"]
DEPENDENCIES = ["uart"]

CONF_PUBLISH_ONLY_ON_FIX = "publish_only_on_fix"

CONF_LATITUDE = "latitude"
CONF_LONGITUDE = "longitude"
CONF_ALTITUDE = "altitude"
CONF_SPEED = "speed"
CONF_COURSE = "course"
CONF_HDOP = "hdop"
CONF_SATELLITES = "satellites"
CONF_FIX = "fix"
CONF_DATE = "date"
CONF_TIME = "time"
CONF_DATETIME = "datetime"

gps_idf_ns = cg.esphome_ns.namespace("gps_idf")
GPSIDFComponent = gps_idf_ns.class_(
    "GPSIDFComponent", cg.PollingComponent, uart.UARTDevice
)

SENSOR_SCHEMA = {
    cv.Optional(CONF_LATITUDE): sensor.sensor_schema(
        unit_of_measurement=UNIT_DEGREES, accuracy_decimals=6
    ),
    cv.Optional(CONF_LONGITUDE): sensor.sensor_schema(
        unit_of_measurement=UNIT_DEGREES, accuracy_decimals=6
    ),
    cv.Optional(CONF_ALTITUDE): sensor.sensor_schema(
        unit_of_measurement=UNIT_METER, accuracy_decimals=1
    ),
    cv.Optional(CONF_SPEED): sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOMETER_PER_HOUR, accuracy_decimals=1
    ),
    cv.Optional(CONF_COURSE): sensor.sensor_schema(
        unit_of_measurement=UNIT_DEGREES, accuracy_decimals=0
    ),
    cv.Optional(CONF_HDOP): sensor.sensor_schema(accuracy_decimals=2),
    cv.Optional(CONF_SATELLITES): sensor.sensor_schema(accuracy_decimals=0),
}

TEXT_SENSOR_SCHEMA = {
    cv.Optional(CONF_DATE): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_TIME): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_DATETIME): text_sensor.text_sensor_schema(),
}

BINARY_SENSOR_SCHEMA = {
    cv.Optional(CONF_FIX): binary_sensor.binary_sensor_schema(),
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(GPSIDFComponent),
            cv.Optional(CONF_PUBLISH_ONLY_ON_FIX, default=True): cv.boolean,
            cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(SENSOR_SCHEMA)
    .extend(TEXT_SENSOR_SCHEMA)
    .extend(BINARY_SENSOR_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_publish_only_on_fix(config[CONF_PUBLISH_ONLY_ON_FIX]))

    # Sensors
    if CONF_LATITUDE in config:
        sens = await sensor.new_sensor(config[CONF_LATITUDE])
        cg.add(var.set_latitude_sensor(sens))
    if CONF_LONGITUDE in config:
        sens = await sensor.new_sensor(config[CONF_LONGITUDE])
        cg.add(var.set_longitude_sensor(sens))
    if CONF_ALTITUDE in config:
        sens = await sensor.new_sensor(config[CONF_ALTITUDE])
        cg.add(var.set_altitude_sensor(sens))
    if CONF_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_SPEED])
        cg.add(var.set_speed_sensor(sens))
    if CONF_COURSE in config:
        sens = await sensor.new_sensor(config[CONF_COURSE])
        cg.add(var.set_course_sensor(sens))
    if CONF_HDOP in config:
        sens = await sensor.new_sensor(config[CONF_HDOP])
        cg.add(var.set_hdop_sensor(sens))
    if CONF_SATELLITES in config:
        sens = await sensor.new_sensor(config[CONF_SATELLITES])
        cg.add(var.set_satellites_sensor(sens))

    # Text sensors
    if CONF_DATE in config:
        ts = await text_sensor.new_text_sensor(config[CONF_DATE])
        cg.add(var.set_date_text_sensor(ts))
    if CONF_TIME in config:
        ts = await text_sensor.new_text_sensor(config[CONF_TIME])
        cg.add(var.set_time_text_sensor(ts))
    if CONF_DATETIME in config:
        ts = await text_sensor.new_text_sensor(config[CONF_DATETIME])
        cg.add(var.set_datetime_text_sensor(ts))

    # Binary sensor
    if CONF_FIX in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_FIX])
        cg.add(var.set_fix_binary_sensor(bs))
