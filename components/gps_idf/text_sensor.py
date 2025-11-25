import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_TYPE
from . import GPS_IDF_CHILD_SCHEMA, CONF_GPS_IDF_ID, gps_idf_ns

DEPENDENCIES = ["gps_idf"]

CONF_DATETIME = "datetime"
CONF_FIX_STATUS = "fix_status"

ICON_CLOCK = "mdi:clock-outline"
ICON_CROSSHAIRS_GPS = "mdi:crosshairs-gps"

TextSensorType = gps_idf_ns.enum("TextSensorType")
TEXT_SENSOR_TYPES = {
    CONF_DATETIME: TextSensorType.DATETIME,
    CONF_FIX_STATUS: TextSensorType.FIX_STATUS,
}

TEXT_SENSOR_DEFAULTS = {
    CONF_DATETIME: {
        "icon": ICON_CLOCK,
    },
    CONF_FIX_STATUS: {
        "icon": ICON_CROSSHAIRS_GPS,
    },
}


def text_sensor_schema(sensor_type):
    defaults = TEXT_SENSOR_DEFAULTS.get(sensor_type, {})
    return text_sensor.text_sensor_schema(**defaults).extend(GPS_IDF_CHILD_SCHEMA)


CONFIG_SCHEMA = cv.typed_schema(
    {
        CONF_DATETIME: text_sensor_schema(CONF_DATETIME),
        CONF_FIX_STATUS: text_sensor_schema(CONF_FIX_STATUS),
    },
    key=CONF_TYPE,
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_GPS_IDF_ID])
    var = await text_sensor.new_text_sensor(config)
    sensor_type = config[CONF_TYPE]
    cg.add(parent.register_text_sensor(var, TEXT_SENSOR_TYPES[sensor_type]))
