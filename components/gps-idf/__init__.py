from esphome.components import sensor, text_sensor, udp, uart, switch
import esphome.config_validation as cv
import esphome.codegen as cg

DEPENDENCIES = ["uart", "wifi", "udp"]
AUTO_LOAD = ["sensor", "text_sensor", "switch"]

nmea_gps_ns = cg.esphome_ns.namespace("gps_idf")
NMEAGPSComponent = nmea_gps_ns.class_(
    "GPSIDFComponent", cg.Component, uart.UARTDevice
)

UDP_BROADCAST_SCHEMA = cv.Schema(
    {
        cv.Optional("enabled", default=False): cv.boolean,
        cv.Optional("port", default=10110): cv.port,
        cv.Optional("broadcast_address", default="255.255.255.255"): cv.string,
        cv.Optional("sentence_filter", default=[]): cv.ensure_list(cv.string),
        cv.Optional("interval", default="15s"): cv.positive_time_period_seconds,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NMEAGPSComponent),
        cv.Required(cv.GenerateID("uart_id")): cv.use_id(uart.UARTComponent),
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
        cv.Optional("udp_broadcast"): UDP_BROADCAST_SCHEMA,
        cv.Optional("udp_broadcast_switch"): switch.switch_schema(
            NMEAGPSComponent,
            icon="mdi:broadcast",
        ),
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
    if "udp_broadcast" in config:
        udp_config = config["udp_broadcast"]
        cg.add(var.set_udp_broadcast_enabled(udp_config["enabled"]))
        cg.add(var.set_udp_broadcast_port(udp_config["port"]))
        cg.add(var.set_udp_broadcast_address(udp_config["broadcast_address"]))
        cg.add(var.set_udp_broadcast_interval(udp_config["interval"].total_milliseconds))
        for sentence in udp_config["sentence_filter"]:
            cg.add(var.add_udp_broadcast_sentence_filter(cg.std_string(sentence)))

    if "udp_broadcast_switch" in config:
        sw = await switch.new_switch(config["udp_broadcast_switch"])
        await cg.register_component(sw, config)
        cg.add(sw.set_write_state_handler(lambda x: var.set_udp_broadcast_enabled(x)))