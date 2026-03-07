import math

from joint_reader_constants import ENCODER_RESOLUTION


def convert_to_radian(encoder_value: float) -> float:
    """Map an encoder value to radians.

    0                   -> -π
    ENCODER_RESOLUTION  ->  π
    """
    return (encoder_value / ENCODER_RESOLUTION) * 2 * math.pi - math.pi
