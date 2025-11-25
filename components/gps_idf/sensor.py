import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_TYPE,
    UNIT_DEGREES,
    UNIT_METER,
    UNIT_KILOMETER_PER_HOUR,
    ICON_EMPTY,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_SPEED,
)
from . import GPS_IDF_CHILD_SCHEMA, CONF_GPS_IDF_ID, gps_idf_ns

DEPENDENCIES = ["gps_idf"]

CONF_LATITUDE = "latitude"
CONF_LONGITUDE = "longitude"
CONF_ALTITUDE = "altitude"
CONF_SPEED = "speed"
CONF_COURSE = "course"
CONF_SATELLITES = "satellites"
CONF_HDOP = "hdop"

ICON_LATITUDE = "mdi:latitude"
ICON_LONGITUDE = "mdi:longitude"
ICON_ALTIMETER = "mdi:altimeter"
ICON_SPEEDOMETER = "mdi:speedometer"
ICON_COMPASS = "mdi:compass"
ICON_SATELLITE = "mdi:satellite-variant"

SensorType = gps_idf_ns.enum("SensorType")
SENSOR_TYPES = {
    CONF_LATITUDE: SensorType.LATITUDE,
    CONF_LONGITUDE: SensorType.LONGITUDE,
    CONF_ALTITUDE: SensorType.ALTITUDE,
    CONF_SPEED: SensorType.SPEED,
    CONF_COURSE: SensorType.COURSE,
    CONF_SATELLITES: SensorType.SATELLITES,
    CONF_HDOP: SensorType.HDOP,
}

SENSOR_DEFAULTS = {
    CONF_LATITUDE: {
        "unit_of_measurement": UNIT_DEGREES,
        "icon": ICON_LATITUDE,
        "accuracy_decimals": 6,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    CONF_LONGITUDE: {
        "unit_of_measurement": UNIT_DEGREES,
        "icon": ICON_LONGITUDE,
        "accuracy_decimals": 6,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    CONF_ALTITUDE: {
        "unit_of_measurement": UNIT_METER,
        "icon": ICON_ALTIMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    CONF_SPEED: {
        "unit_of_measurement": UNIT_KILOMETER_PER_HOUR,
        "icon": ICON_SPEEDOMETER,
        "accuracy_decimals": 1,
        "device_class": DEVICE_CLASS_SPEED,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    CONF_COURSE: {
        "unit_of_measurement": UNIT_DEGREES,
        "icon": ICON_COMPASS,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    CONF_SATELLITES: {
        "icon": ICON_SATELLITE,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    CONF_HDOP: {
        "icon": ICON_EMPTY,
        "accuracy_decimals": 2,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
}


def sensor_schema(sensor_type):
    defaults = SENSOR_DEFAULTS.get(sensor_type, {})
    return sensor.sensor_schema(**defaults).extend(GPS_IDF_CHILD_SCHEMA)


CONFIG_SCHEMA = cv.typed_schema(
    {
        CONF_LATITUDE: sensor_schema(CONF_LATITUDE),
        CONF_LONGITUDE: sensor_schema(CONF_LONGITUDE),
        CONF_ALTITUDE: sensor_schema(CONF_ALTITUDE),
        CONF_SPEED: sensor_schema(CONF_SPEED),
        CONF_COURSE: sensor_schema(CONF_COURSE),
        CONF_SATELLITES: sensor_schema(CONF_SATELLITES),
        CONF_HDOP: sensor_schema(CONF_HDOP),
    },
    key=CONF_TYPE,
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_GPS_IDF_ID])
    var = await sensor.new_sensor(config)
    sensor_type = config[CONF_TYPE]
    cg.add(parent.register_sensor(var, SENSOR_TYPES[sensor_type]))
