import array, time
import random
import socket

NUM_LEDS = 200
MCAST_GRP = "239.1.1.1"
MCAST_PORT = 4545
global_dimming = 255

led_values = [(0, 0, 0)] * NUM_LEDS


def pixels_show(sm):
    leds_packed = array.array("I", [0 for _ in range(NUM_LEDS)])
    for i, pixel in enumerate(led_values):
        adj_red = ((pixel[0] ** 2) * global_dimming) // 255**2
        adj_green = ((pixel[1] ** 2) * global_dimming) // 255**2
        adj_blue = ((pixel[2] ** 2) * global_dimming) // 255**2
        leds_packed[i] = (adj_blue << 16) + (adj_red << 8) + adj_green
    sm.put(leds_packed, 8)
    time.sleep_us(100)


def pixels_set(i, val):
    led_values[i] = val


def pixels_fill(color):
    for i in range(len(led_values)):
        pixels_set(i, color)


def HSV2RGB(h, s, v):
    """
    Convert HSV to RGB color space.

    Courtesy ChatGPT, because MircoPython does not provide colorsys.
    TODO: Convert to all integer math

    Parameters:
    h (float): Hue, should be in [0, 360)
    s (float): Saturation, should be in [0, 1]
    v (float): Value, should be in [0, 1]

    Returns:
    (int, int, int): Tuple of RGB values, each in [0, 255]
    """
    if s == 0.0:
        v *= 255
        return (int(v), int(v), int(v))

    h /= 60
    i = int(h)
    f = h - i
    p = v * (1 - s)
    q = v * (1 - s * f)
    t = v * (1 - s * (1 - f))

    v *= 255
    p *= 255
    q *= 255
    t *= 255

    if i == 0:
        return (int(v), int(t), int(p))
    elif i == 1:
        return (int(q), int(v), int(p))
    elif i == 2:
        return (int(p), int(v), int(t))
    elif i == 3:
        return (int(p), int(q), int(v))
    elif i == 4:
        return (int(t), int(p), int(v))
    else:
        return (int(v), int(p), int(q))


def inet_aton(addr):
    return bytes(map(int, addr.split(".")))


def main(sm, ip):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(
        socket.IPPROTO_IP,
        socket.IP_ADD_MEMBERSHIP,
        inet_aton(MCAST_GRP) + inet_aton(ip),
    )
    sock.bind(("", MCAST_PORT))

    try:
        while True:
            data, _ = sock.recvfrom(16)
            spell = data.decode()
            action = SPELL_TO_ACTION.get(spell, 0)
            if callable(action):
                action(sm, ip)
            else:
                (r, g, b) = HSV2RGB(action, 1, 1)
                pixels_fill((r, g, b))
                pixels_show(sm)
    finally:
        sock.close()


def main2(sm, ip):
    while True:
        pixels_fill((0, 0, 0))
        prev_choice = False
        for led in range(NUM_LEDS):
            if (
                random.choice([True, False, False, False, False, False, False])
                and not prev_choice
            ):
                prev_choice = True
                pixels_set(led, (255, 138, 18))
            else:
                prev_choice = False
                pixels_set(led, (0, 0, 30))
        pixels_show(sm)


def main3(sm, ip):
    while True:
        for hue in range(360):
            pixels_fill(HSV2RGB(hue, 1, 1))
            pixels_show(sm)
            time.sleep(0.05)


def main4(sm, ip):
    MIN_VAL = 40
    MAX_VAL = 255
    STEP = 50
    SAT = 0.5
    BASE_HUE = 0
    ACCENT_HUE = 30
    denominator = 255
    prev_dirs = [False] * NUM_LEDS
    base_color = HSV2RGB(BASE_HUE, SAT, 0 / denominator)
    weight = 0
    while True:
        weight += 1
        dirs = [random.choice([True] + ([False] * weight)) for i in range(NUM_LEDS)]
        if not any(dirs):
            break
        for i, p in enumerate(prev_dirs):
            if p:
                dirs[i] = False
        for current_bright in range(MIN_VAL, MAX_VAL, STEP):
            up_color = HSV2RGB(ACCENT_HUE, SAT, current_bright / denominator)
            down_color = HSV2RGB(
                ACCENT_HUE, SAT, (MAX_VAL - (current_bright - MIN_VAL)) / denominator
            )
            for i in range(NUM_LEDS):
                if dirs[i]:
                    pixels_set(i, up_color)
                elif prev_dirs[i]:
                    pixels_set(i, down_color)
                else:
                    pixels_set(i, base_color)
            pixels_show(sm)
        prev_dirs = dirs


main = main

SPELL_TO_ACTION = {"incendio": main4, "mimblewimble": 120, "descendo": 240}

# Note webserver must be running for this to work:
# python3 -mhttp.server 4545
if __name__ == "__main__":
    print("Test mode.  Entering bootloader... ")
    # Next 2 lines clear PIO memory to keep repeated
    # re-runs causing the PIO to run out of memory
    import rp2

    rp2.PIO(0).remove_program()
    from main import bootloader

    bootloader()
