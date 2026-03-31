import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, remote_transmitter, remote_receiver, sensor
from esphome.const import CONF_ID, CONF_SENSOR, CONF_RECEIVER_ID

DEPENDENCIES = ["climate", "remote_transmitter"]

motech_ac_ns = cg.esphome_ns.namespace("motech_ac")
MotechACClimate = motech_ac_ns.class_(
    "MotechACClimate", climate.Climate, cg.Component
)

CONF_TRANSMITTER_ID = "transmitter_id"

CONFIG_SCHEMA = climate.climate_schema(MotechACClimate).extend(
    {
        cv.Required(CONF_TRANSMITTER_ID): cv.use_id(
            remote_transmitter.RemoteTransmitterComponent
        ),
        cv.Optional(CONF_RECEIVER_ID): cv.use_id(
            remote_receiver.RemoteReceiverComponent
        ),
        cv.Optional(CONF_SENSOR): cv.use_id(sensor.Sensor),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    tx = await cg.get_variable(config[CONF_TRANSMITTER_ID])
    cg.add(var.set_transmitter(tx))

    if CONF_RECEIVER_ID in config:
        rcvr = await cg.get_variable(config[CONF_RECEIVER_ID])
        cg.add(rcvr.register_listener(var))

    if CONF_SENSOR in config:
        sens = await cg.get_variable(config[CONF_SENSOR])
        cg.add(var.set_sensor(sens))
