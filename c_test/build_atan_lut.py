#!/usr/bin/env python

import numpy as np
from math import atan

resolution = 64
lut = [0]

for i in range(1, resolution + 1):
    lut.append(int(np.degrees(atan(i / resolution))))

with open("atan_lut.hpp", "w") as f:
    f.write("#ifndef __ATAN_LUT_H__\n")
    f.write("#define __ATAN_LUT_H__ 1\n")
    f.write("\n")
    f.write("#include <vector>\n")
    f.write("\n")

    f.write(f"#define LUT_SIZE {resolution}\n")
    f.write(f"#define atan_lut atan_lut_{resolution}\n")
    f.write("\n")
    f.write(f"std::vector<int> atan_lut_{resolution} = {{\n")

    for i in lut:
        f.write(f"    {i},\n")

    f.write("};\n")
    f.write("\n")
    f.write("#endif // __ATAN_LUT_H__\n")
