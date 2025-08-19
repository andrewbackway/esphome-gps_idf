# my_components/gps_tiny/__init__.py
cv.Schema(
{
cv.GenerateID(): cv.declare_id(GPSTinyComponent),
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