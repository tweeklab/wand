from typing import NamedTuple, Tuple


class NetConfig(NamedTuple):
    conv_layers: int
    conv_per_layer: int
    conv_kernel_shape: Tuple[int, int]
    dropout1: float
    dropout2: float
    dense_size: int
