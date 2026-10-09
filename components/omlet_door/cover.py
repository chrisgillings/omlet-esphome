from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import cover, voltage_sampler
import esphome.config_validation as cv
from esphome.const import (
    CONF_CLOSE_DURATION,
    CONF_ID,
    CONF_MAX_DURATION,
    CONF_OPEN_DURATION,
)
from esphome.types import ConfigType

from . import omlet_door_ns

AUTO_LOAD = ["voltage_sampler"]

CONF_HIGH_SIDE_A = "high_side_a"
CONF_LOW_SIDE_A = "low_side_a"
CONF_HIGH_SIDE_B = "high_side_b"
CONF_LOW_SIDE_B = "low_side_b"
CONF_CURRENT_SAMPLER = "current_sampler"
CONF_CURRENT_ENABLE_PIN = "current_enable_pin"
CONF_CRUSH_PIN = "crush_pin"
CONF_INVERT_DIRECTION = "invert_direction"
CONF_START_SENSING_DELAY = "start_sensing_delay"
CONF_STALL_TIME = "stall_time"
CONF_STALL_THRESHOLD = "stall_threshold"
CONF_STALL_RATIO = "stall_ratio"
CONF_MIN_CURRENT = "min_current"
CONF_MAX_CRUSH_RETRIES = "max_crush_retries"
CONF_CRUSH_RETRY_DELAY = "crush_retry_delay"

OmletDoor = omlet_door_ns.class_("OmletDoor", cover.Cover, cg.Component)
CalibrateAction = omlet_door_ns.class_("CalibrateAction", automation.Action)

CONFIG_SCHEMA = (
    cover.cover_schema(OmletDoor, device_class="door")
    .extend(
        {
            cv.Required(CONF_HIGH_SIDE_A): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_LOW_SIDE_A): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_HIGH_SIDE_B): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_LOW_SIDE_B): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_CURRENT_SAMPLER): cv.use_id(
                voltage_sampler.VoltageSampler
            ),
            cv.Optional(CONF_CURRENT_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_CRUSH_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
            cv.Optional(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_MAX_DURATION, default="30s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_START_SENSING_DELAY, default="300ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_STALL_TIME, default="100ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_STALL_THRESHOLD): cv.float_range(
                min=0, min_included=False
            ),
            cv.Optional(CONF_STALL_RATIO, default=2.0): cv.float_range(min=1.1, max=10),
            cv.Optional(CONF_MAX_CRUSH_RETRIES, default=3): cv.int_range(min=0, max=10),
            cv.Optional(
                CONF_CRUSH_RETRY_DELAY, default="10s"
            ): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config: ConfigType) -> None:
    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    for key, setter in (
        (CONF_HIGH_SIDE_A, var.set_high_side_a),
        (CONF_LOW_SIDE_A, var.set_low_side_a),
        (CONF_HIGH_SIDE_B, var.set_high_side_b),
        (CONF_LOW_SIDE_B, var.set_low_side_b),
    ):
        pin = await cg.gpio_pin_expression(config[key])
        cg.add(setter(pin))

    sampler = await cg.get_variable(config[CONF_CURRENT_SAMPLER])
    cg.add(var.set_current_sampler(sampler))

    if (enable_pin := config.get(CONF_CURRENT_ENABLE_PIN)) is not None:
        pin = await cg.gpio_pin_expression(enable_pin)
        cg.add(var.set_current_enable_pin(pin))

    if (crush_pin := config.get(CONF_CRUSH_PIN)) is not None:
        pin = await cg.gpio_pin_expression(crush_pin)
        cg.add(var.set_crush_pin(pin))

    cg.add(var.set_invert_direction(config[CONF_INVERT_DIRECTION]))
    if (open_duration := config.get(CONF_OPEN_DURATION)) is not None:
        cg.add(var.set_open_duration(open_duration))
    if (close_duration := config.get(CONF_CLOSE_DURATION)) is not None:
        cg.add(var.set_close_duration(close_duration))
    cg.add(var.set_max_duration(config[CONF_MAX_DURATION]))
    cg.add(var.set_start_sensing_delay(config[CONF_START_SENSING_DELAY]))
    cg.add(var.set_stall_time(config[CONF_STALL_TIME]))
    if (threshold := config.get(CONF_STALL_THRESHOLD)) is not None:
        cg.add(var.set_stall_threshold(threshold))
    cg.add(var.set_stall_ratio(config[CONF_STALL_RATIO]))
    if (min_current := config.get(CONF_MIN_CURRENT)) is not None:
        cg.add(var.set_min_current(min_current))
    cg.add(var.set_max_crush_retries(config[CONF_MAX_CRUSH_RETRIES]))
    cg.add(var.set_crush_retry_delay(config[CONF_CRUSH_RETRY_DELAY]))


@automation.register_action(
    "omlet_door.calibrate",
    CalibrateAction,
    automation.maybe_simple_id({cv.Required(CONF_ID): cv.use_id(OmletDoor)}),
    synchronous=True,
)
async def calibrate_action_to_code(
    config: ConfigType, action_id: cg.ID, template_arg: cg.TemplateArguments, args
) -> cg.MockObj:
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)
