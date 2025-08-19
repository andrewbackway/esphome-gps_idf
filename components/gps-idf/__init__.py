from esphome.components import sensor, text_sensor, uart
import esphome.config_validation as cv
import esphome.codegen as cg

DEPENDENCIES = ["uart", "sensor", "text_sensor"]
AUTO_LOAD = []

nmea_gps_ns = cg.esphome_ns.namespace("nmea_gps")
NMEAGPSComponent = nmea_gps_ns.class_(
    "NMEAGPSComponent", cg.Component, uart.UARTDevice
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NMEAGPSComponent),
        cv.Required(cv.GenerateID("uart_id")): cv.use_id(uart.UARTComponent),
        cv.Optional("verbose_logging", default=False): cv.boolean,
        cv.Optional("latitude"): sensor.sensor_schema(
            unit_of_measurement="°",
            accuracy_decimals=6,
        ),
        cv.Optional("longitude"): sensor.sensor_schema(
            unit_of_measurement="°",
            accuracy_decimals=6,
        ),
        cv.Optional("altitude"): sensor.sensor_schema(
            unit_of_measurement="m",
            accuracy_decimals=1,
        ),
        cv.Optional("speed"): sensor.sensor_schema(
            unit_of_measurement="km/h",
            accuracy_decimals=1,
        ),
        cv.Optional("course"): sensor.sensor_schema(
            unit_of_measurement="°",
            accuracy_decimals=1,
        ),
        cv.Optional("satellites"): sensor.sensor_schema(
            accuracy_decimals=0,
        ),
        cv.Optional("hdop"): sensor.sensor_schema(
            accuracy_decimals=2,
        ),
        cv.Optional("datetime"): text_sensor.text_sensor_schema(),
        cv.Optional("fix_status"): text_sensor.text_sensor_schema(),
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[cv.GenerateID()])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if "latitude" in config:
        sens = await sensor.new_sensor(config["latitude"])
        cg.add(var.set_latitude_sensor(sens))
    if "longitude" in config:
        sens = await sensor.new_sensor(config["longitude"])
        cg.add(var.set_longitude_sensor(sens))
    if "altitude" in config:
        sens = await sensor.new_sensor(config["altitude"])
        cg.add(var.set_altitude_sensor(sens))
    if "speed" in config:
        sens = await sensor.new_sensor(config["speed"])
        cg.add(var.set_speed_sensor(sens))
    if "course" in config:
        sens = await sensor.new_sensor(config["course"])
        cg.add(var.set_course_sensor(sens))
    if "satellites" in config:
        sens = await sensor.new_sensor(config["satellites"])
        cg.add(var.set_satellites_sensor(sens))
    if "hdop" in config:
        sens = await sensor.new_sensor(config["hdop"])
        cg.add(var.set_hdop_sensor(sens))
    if "datetime" in config:
        sens = await text_sensor.new_text_sensor(config["datetime"])
        cg.add(var.set_datetime_sensor(sens))
    if "fix_status" in config:
        sens = await text_sensor.new_text_sensor(config["fix_status"])
        cg.add(var.set_fix_status_sensor(sens))
    if "verbose_logging" in config:
        cg.add(var.set_verbose_logging(config["verbose_logging"]))