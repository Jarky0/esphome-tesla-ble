import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, binary_sensor, button, lock, sensor, switch, text_sensor, web_server_base
from esphome.const import CONF_ID, CONF_MAC_ADDRESS, CONF_NAME, DEVICE_CLASS_EMPTY, DEVICE_CLASS_LOCK, DEVICE_CLASS_PRESENCE, DEVICE_CLASS_OPENING
from esphome.core import CORE

CODEOWNERS = ["@yoziru"]
DEPENDENCIES = ["ble_client", "web_server_base"]

tesla_ble_vehicle_ns = cg.esphome_ns.namespace("tesla_ble_vehicle")
TeslaBLEVehicle = tesla_ble_vehicle_ns.class_(
    "TeslaBLEVehicle", cg.PollingComponent, ble_client.BLEClientNode
)
CustomBinarySensor = tesla_ble_vehicle_ns.class_("CustomBinarySensor", binary_sensor.BinarySensor)

AUTO_LOAD = ["binary_sensor", "button", "lock", "sensor", "switch", "text_sensor"]
CONF_VIN = "vin"
CONF_IS_ASLEEP = "is_asleep"
CONF_IS_UNLOCKED = "is_unlocked"
CONF_IS_USER_PRESENT = "is_user_present"
CONF_IS_CHARGE_FLAP_OPEN = "is_charge_flap_open"
CONF_ENABLE_HTTP_PROXY = "enable_http_proxy"
CONF_HTTP_PROXY_PORT = "http_proxy_port"
CONF_PRIVATE_KEY = "private_key"
CONF_PUBLIC_KEY = "public_key"
CONF_SCAN_DURATION = "scan_duration"
CONF_SCAN_INTERVAL = "scan_interval"
CONF_SCAN_WINDOW = "scan_window"
CONF_CONNECTION_TIMEOUT = "connection_timeout"
CONF_COMMAND_TIMEOUT = "command_timeout"
CONF_DISCONNECT_TIMEOUT = "disconnect_timeout"
CONF_HTTP_PROXY_ENABLED = "http_proxy_enabled"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TeslaBLEVehicle),
            cv.Required(CONF_VIN): cv.string,
            cv.Required(CONF_PRIVATE_KEY): cv.string,
            cv.Optional(CONF_PUBLIC_KEY): cv.string,
            cv.Optional(CONF_IS_ASLEEP): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_EMPTY,
                icon="mdi:sleep",
            ),
            cv.Optional(CONF_IS_UNLOCKED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_LOCK,
                icon="mdi:car-door-lock",
            ),
            cv.Optional(CONF_IS_USER_PRESENT): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PRESENCE,
                icon="mdi:account",
            ),
            cv.Optional(CONF_IS_CHARGE_FLAP_OPEN): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OPENING,
                icon="mdi:ev-station",
            ),
            cv.Optional("door_lock"): lock.LOCK_SCHEMA,
            cv.Optional("honk_button"): button.button_schema(TeslaBLEVehicle),
            cv.Optional("climate_switch"): switch.switch_schema(TeslaBLEVehicle),
            cv.Optional(CONF_HTTP_PROXY_ENABLED, default=False): cv.boolean,
            cv.Optional(CONF_HTTP_PROXY_PORT, default=8088): cv.port,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(ble_client.BLE_CLIENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    await ble_client.register_ble_node(var, config)

    cg.add(var.set_vin(config[CONF_VIN]))
    cg.add(var.set_private_key_b64(config[CONF_PRIVATE_KEY]))

    if CONF_PUBLIC_KEY in config:
        cg.add(var.set_public_key_str(config[CONF_PUBLIC_KEY]))

    if CONF_IS_ASLEEP in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_IS_ASLEEP])
        cg.add(var.set_binary_sensor_is_asleep(sens))

    if CONF_IS_UNLOCKED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_IS_UNLOCKED])
        cg.add(var.set_binary_sensor_is_unlocked(sens))

    if CONF_IS_USER_PRESENT in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_IS_USER_PRESENT])
        cg.add(var.set_binary_sensor_is_user_present(sens))

    if CONF_IS_CHARGE_FLAP_OPEN in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_IS_CHARGE_FLAP_OPEN])
        cg.add(var.set_binary_sensor_is_charge_flap_open(sens))

    if "door_lock" in config:
        lock_ = await lock.new_lock(config["door_lock"])
        cg.add(var.set_lock_door(lock_))

    if "honk_button" in config:
        button_ = await button.new_button(config["honk_button"])
        cg.add(var.set_button_honk(button_))

    if "climate_switch" in config:
        switch_ = await switch.new_switch(config["climate_switch"])
        cg.add(var.set_switch_climate(switch_))

    if config[CONF_HTTP_PROXY_ENABLED]:
        cg.add_define("USE_WEB_SERVER")
        cg.add(var.set_http_proxy_enabled(True))
        cg.add(var.set_http_proxy_port(config[CONF_HTTP_PROXY_PORT]))

    cg.add_library("noise-c", "0.1.6")
    cg.add_library("ArduinoJson", "6.18.5")
