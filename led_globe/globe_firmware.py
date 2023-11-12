import array, time
import random
import socket
import json
import _thread

NUM_LEDS = 200
MCAST_GRP = "239.1.1.1"
MCAST_PORT = 4545
led_run = True
abort_flag_lock = _thread.allocate_lock()
abort_flag = False
pending_command_lock = _thread.allocate_lock()
pending_commands = []
run_lock = _thread.allocate_lock()
global_dimming = 255

led_values = [(0, 0, 0)] * NUM_LEDS

def sparkle(sm):
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
    while not abort_flag:
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

ROUTINES = {
    'sparkle': sparkle
}

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

def push_pending_command(command):
    pending_command_lock.acquire()
    pending_commands.insert(0, command)
    pending_command_lock.release()

def pop_pending_command():
    pending_command_lock.acquire()
    if len(pending_commands) == 0:
        ps = None
    else:
        ps = pending_commands.pop()
    pending_command_lock.release()
    return ps

def peek_pending_command():
    pending_command_lock.acquire()
    ps = pending_commands[-1]
    pending_command_lock.release()
    return ps

def clear_pending_commands(abort = False):
    pending_command_lock.acquire()
    pending_commands.clear()
    if abort:
        abort_current_command()
    pending_command_lock.release()

def abort_current_command():
    global abort_flag
    abort_flag_lock.acquire()
    abort_flag = True
    abort_flag_lock.release()

def clear_abort_flag():
    global abort_flag
    abort_flag_lock.acquire()
    abort_flag = False
    abort_flag_lock.release()

def led_state_thread(sm):
    print("Starting")
    while led_run:
        time.sleep_ms(500)
        command = pop_pending_command()
        if command is None:
            continue
        clear_abort_flag()
        try:
            run_lock.acquire()
            action = command['action']
            if action == 'sethsv':
                (r, g, b) = HSV2RGB(command['h'], command['s'], command['v'])
                pixels_fill((r,g,b))
                pixels_show(sm)
            elif action == 'play':
                routine = command['func']
                if routine in ROUTINES:
                    func = ROUTINES[routine]
                    func(sm)
                else:
                    print(f"play: unknown routine: {routine}")
            else:
                print(f"Unknown action: {action}")
        except Exception as e:
            print(f"Bad command: {command}: {e}")
        finally:
            run_lock.release()

    print("Exiting")

def main(sm, ip):
    global led_run
    pixels_fill((0, 0, 0))
    pixels_show(sm)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(
        socket.IPPROTO_IP,
        socket.IP_ADD_MEMBERSHIP,
        inet_aton(MCAST_GRP) + inet_aton(ip),
    )
    sock.bind(("", MCAST_PORT))

    _thread.start_new_thread(led_state_thread, [sm])

    try:
        while True:
            data, _ = sock.recvfrom(64)
            try:
                command = json.loads(data.decode())
                action = command['action']
                if action == 'abort':
                    clear_pending_commands(abort=True)
                    if command.get('blank', False):
                        run_lock.acquire()
                        pixels_fill((0, 0, 0))
                        pixels_show(sm)
                        run_lock.release()
                else:
                    push_pending_command(command)
            except Exception as e:
                print(f"Bad json parse: {e}")
    finally:
        sock.close()
        led_run = False


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
