from typing import NamedTuple, Tuple


class NetConfig(NamedTuple):
    conv_per_layer1: int
    conv_kernel_shape1: Tuple[int, int]
    conv_kernel_shape2: Tuple[int, int]
    dropout1: float
    dense_size: int
