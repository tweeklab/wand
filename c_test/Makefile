CXXFLAGS = --std=c++17 # -O3
CPPFLAGS = -I/usr/include/eigen3

all: test filter

line: line.cpp
test: blob_rect.cpp point.cpp test.cpp
filter: blob_rect.cpp point.cpp filter.cpp test_frames.hpp atan_lut.hpp cos_lut.hpp
atan_lut.hpp: build_atan_lut.py
	python ./build_atan_lut.py
cos_lut.hpp: build_cos_lut.py
	python ./build_cos_lut.py


clean:
	rm -f test filter line atan_lut.hpp cost_lut.hpp
