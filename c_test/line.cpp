#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>

using namespace std;

#define FINAL_IMAGE_SIZE 40*29

uint8_t *image = (uint8_t *)malloc(FINAL_IMAGE_SIZE);

// Adapted from the Adafruit GFX library:
// https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_GFX.cpp#L132
// void draw_line(uint8_t *buf, int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
//     int16_t tmp;
//     int16_t steep = abs(y1 - y0) > abs(x1 - x0);
//     if (steep)
//     {
//         tmp = x0;
//         x0 = y0;
//         y0 = tmp;

//         tmp = x1;
//         x1 = y1;
//         y1 = tmp;
//     }

//     if (x0 > x1)
//     {
//         tmp = x0;
//         x0 = x1;
//         x1 = tmp;

//         tmp = y0;
//         y0 = y1;
//         y1 = tmp;
//     }

//     int16_t dx, dy;
//     dx = x1 - x0;
//     dy = abs(y1 - y0);

//     int16_t err = dx / 2;
//     int16_t ystep;

//     if (y0 < y1)
//     {
//         ystep = 1;
//     }
//     else
//     {
//         ystep = -1;
//     }

//     for (; x0 <= x1; x0++)
//     {
//         if (steep)
//         {
//             buf[y0 + (x0*40)] = 255;
//         }
//         else
//         {
//             buf[x0 + (y0*40)] = 255;
//         }
//         err -= dy;
//         if (err < 0)
//         {
//             y0 += ystep;
//             err += dx;
//         }
//     }
// }

// Taken basically verbatim from: https://stackoverflow.com/a/14506390
void draw_line(uint8_t *buf, int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sgnX = x0 < x1 ? 1 : -1;
    int sgnY = y0 < y1 ? 1 : -1;
    int e = 0;
    for (int i=0; i < dx+dy; i++) {
        buf[x0 + (y0*40)] = 255;
        int e1 = e + dy;
        int e2 = e - dx;
        if (abs(e1) < abs(e2)) {
            x0 += sgnX;
            e = e1;
        } else {
            y0 += sgnY;
            e = e2;
        }
    }
}

int main() {
    ofstream f;
    f.open("image.bin", ios::binary|ios::trunc|ios::out);
    draw_line(
        image,
        10, 30,
        30, 1
    );
    draw_line(
        image,
        30, 1,
        30,20
    );
    draw_line(
        image,
        30,20,
        39,29
    );
    draw_line(
        image,
        39,28,
        25,28
    );
    draw_line(
        image,
        25,28,
        10,10
    );
    draw_line(
        image,
        10,10,
        0,20
    );

    f.write((const char *)image, FINAL_IMAGE_SIZE);

    f.close();

    return 0;
}