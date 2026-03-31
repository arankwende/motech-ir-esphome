import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, remote_transmitter
from esphome.const import (
    CONF_ID,
)

AUTO_LOAD = ["climate"]
DEPENDENCIES = ["remote_transmitter"]

motech_ac_ns = cg.esphome_ns.namespace("motech_ac")
MotechACClimate = motech_ac_ns.class_(
    "MotechACClimate", climate.Climate, cg.Component
)

CONF_TRANSMITTER_ID = "transmitter_id"

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(MotechACClimate),
        cv.Required(CONF_TRANSMITTER_ID): cv.use_id(
            remote_transmitter.RemoteTransmitterComponent
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    transmitter = await cg.get_variable(config[CONF_TRANSMITTER_ID])
    cg.add(var.set_transmitter(transmitter))
