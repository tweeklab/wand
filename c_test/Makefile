CXXFLAGS = --std=c++17 # -O3
CPPFLAGS = -I/usr/include/eigen3

all: test filter

test: blob_rect.cpp test.cpp
filter: blob_rect.cpp filter.cpp test_frames.hpp atan_lut.hpp
atan_lut.hpp: build_atan_lut.py
	python ./build_atan_lut.py

clean:
	rm -f test filter atan_lut.hpp