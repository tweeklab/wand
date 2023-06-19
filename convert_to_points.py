#!/usr/bin/env python

import json

header = []
header.append("#ifndef __TEST_FRAMES_HPP__")
header.append("#define __TEST_FRAMES_HPP__ 1")
header.append("#include <vector>")
header.append('#include "blob_rect.hpp"')
header.append("")
header.append("")

with open("frames_dump.json") as f:
    frames = json.load(f)

header.append("std::vector<std::vector<Point>> test_frame_stream = {")

print(len(frames))
for frame in frames:
    header.append("    {")
    for point in frame:
        header.append(f"        Point({point[0]}, {point[1]}),")
    header.append("    },")

header.append("};")
header.append("#endif // __TEST_FRAMES_HPP__")

with open("c_test/test_frames.hpp", "w") as f:
    for line in header:
        f.write(line + "\n")
