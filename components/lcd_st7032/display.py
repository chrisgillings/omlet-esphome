from esphome import pins
import esphome.codegen as cg
from esphome.components import i2c, lcd_base
import esphome.config_validation as cv
from esphome.const import (
    CONF_BACKLIGHT_PIN,
    CONF_CONTRAST,
    CONF_ENABLE_PIN,
    CONF_ID,
    CONF_LAMBDA,
    CONF_RESET_PIN,
)
from esphome.types import ConfigType

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["lcd_base"]

lcd_st7032_ns = cg.esphome_ns.namespace("lcd_st7032")
ST7032LCDDisplay = lcd_st7032_ns.class_(
    "ST7032LCDDisplay", lcd_base.LCDDisplay, i2c.I2CDevice
)

CONFIG_SCHEMA = lcd_base.LCD_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ST7032LCDDisplay),
        cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_BACKLIGHT_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_CONTRAST, default=22): cv.int_range(min=0, max=63),
    }
).extend(i2c.i2c_device_schema(0x3E))


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await lcd_base.setup_lcd_display(var, config)
    await i2c.register_i2c_device(var, config)

    for key, setter in (
        (CONF_RESET_PIN, var.set_reset_pin),
        (CONF_ENABLE_PIN, var.set_enable_pin),
        (CONF_BACKLIGHT_PIN, var.set_backlight_pin),
    ):
        if (pin_config := config.get(key)) is not None:
            pin = await cg.gpio_pin_expression(pin_config)
            cg.add(setter(pin))
    cg.add(var.set_contrast(config[CONF_CONTRAST]))

    if CONF_LAMBDA in config:
        lambda_ = await cg.process_lambda(
            config[CONF_LAMBDA],
            [(ST7032LCDDisplay.operator("ref"), "it")],
            return_type=cg.void,
        )
        cg.add(var.set_writer(lambda_))
