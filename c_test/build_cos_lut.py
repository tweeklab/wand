#!/usr/bin/env python

import numpy as np
from math import cos

degrees_max = 45
lut = [32768]

for i in range(1, degrees_max + 1):
    lut.append(int(cos(np.radians(i)) * 32768))

with open("cos_lut.hpp", "w") as f:
    f.write("#ifndef __COS_LUT_H__\n")
    f.write("#define __COS_LUT_H__ 1\n")
    f.write("\n")
    f.write("#include <vector>\n")
    f.write("\n")

    f.write(f"#define COS_LUT_SIZE {degrees_max+1}\n")
    f.write(f"#define cos_lut cos_lut_{degrees_max}\n")
    f.write("\n")
    f.write(f"std::vector<int> cos_lut_{degrees_max} = {{\n")

    for i in lut:
        f.write(f"    {i},\n")

    f.write("};\n")
    f.write("\n")
    f.write("#endif // __COS_LUT_H__\n")
