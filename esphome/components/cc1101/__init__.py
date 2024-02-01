import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi, voltage_sampler
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_INVERTED,
    CONF_MODE,
    CONF_NUMBER,
    CONF_OUTPUT,
)

from esphome import pins

AUTO_LOAD = ["voltage_sampler"]
DEPENDENCIES = ["spi"]
CODEOWNERS = ["@Mat931"]

cc1101_ns = cg.esphome_ns.namespace("cc1101")
CC1101Component = cc1101_ns.class_("CC1101Component", cg.Component, spi.SPIDevice)
CC1101GPIOPin = cc1101_ns.class_(
    "CC1101GPIOPin", cg.GPIOPin, cg.Parented.template(CC1101Component)
)

GDOModeOptions = cc1101_ns.enum("GDOModeOptions")
GDO0_MODE_OPTIONS = {
    "auto": GDOModeOptions.GDO_MODE_AUTO,
    "disabled": GDOModeOptions.GDO_MODE_DISABLED,
    "gpio": GDOModeOptions.GDO_MODE_GPIO,
    "async_serial_rx_tx": GDOModeOptions.GDO_MODE_ASYNC_SERIAL_RX_TX,
    "async_serial_rx": GDOModeOptions.GDO_MODE_ASYNC_SERIAL_RX,
    "async_serial_tx": GDOModeOptions.GDO_MODE_ASYNC_SERIAL_TX,
    "analog_temperature_out": GDOModeOptions.GDO_MODE_ANALOG_TEMPERATURE_OUT,
}
GDO2_MODE_OPTIONS = {
    "auto": GDOModeOptions.GDO_MODE_AUTO,
    "disabled": GDOModeOptions.GDO_MODE_DISABLED,
    "gpio": GDOModeOptions.GDO_MODE_GPIO,
    "async_serial_rx": GDOModeOptions.GDO_MODE_ASYNC_SERIAL_RX,
}

CONF_GDO0_PIN = "gdo0_pin"
CONF_GDO2_PIN = "gdo2_pin"
CONF_GDO0_ADC_ID = "gdo0_adc_id"
CONF_GDO0_PIN_MODE = "gdo0_pin_mode"
CONF_GDO2_PIN_MODE = "gdo2_pin_mode"
CONF_CC1101 = "cc1101"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CC1101Component),
            cv.Optional(CONF_GDO0_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_GDO2_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_GDO0_ADC_ID): cv.use_id(voltage_sampler.VoltageSampler),
            cv.Optional(CONF_GDO0_PIN_MODE, default="AUTO"): cv.enum(
                GDO0_MODE_OPTIONS, lower=True
            ),
            cv.Optional(CONF_GDO2_PIN_MODE, default="AUTO"): cv.enum(
                GDO2_MODE_OPTIONS, lower=True
            ),
        }
    )
    .extend(spi.spi_device_schema(cs_pin_required=True))
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    gdo0_pin = await cg.gpio_pin_expression(config[CONF_GDO0_PIN])
    cg.add(var.set_gdo0_pin(gdo0_pin))
    gdo0_adc_id = await cg.get_variable(config[CONF_GDO0_ADC_ID])
    cg.add(var.set_gdo0_adc(gdo0_adc_id))
    gdo2_pin = await cg.gpio_pin_expression(config[CONF_GDO2_PIN])
    cg.add(var.set_gdo2_pin(gdo2_pin))


def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value


CC1101_PIN_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.declare_id(CC1101GPIOPin),
        cv.Required(CONF_CC1101): cv.use_id(CC1101Component),
        cv.Required(CONF_NUMBER): cv.one_of(0, 2, int=True),
        cv.Optional(CONF_MODE, default={}): cv.All(
            {
                cv.Optional(CONF_INPUT, default=False): cv.boolean,
                cv.Optional(CONF_OUTPUT, default=False): cv.boolean,
            },
            validate_mode,
        ),
        cv.Optional(CONF_INVERTED, default=False): cv.boolean,
    }
)


@pins.PIN_SCHEMA_REGISTRY.register("cc1101", CC1101_PIN_SCHEMA)
async def cc1101_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_CC1101])

    cg.add(var.set_parent(parent))

    num = config[CONF_NUMBER]
    cg.add(var.set_pin(num))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    return var
