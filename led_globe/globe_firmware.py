import array, time
import random
import socket
import select
import json
import _thread
import machine
import ubinascii
import network
import urequests
import math
import sys

CONTROLLER_BASE_URL = "http://10.0.0.30:4545"
DEVICE_CONFIG = {}
DEFAULT_DEVICE_CONFIG = {
    'real_num_leds': 200,
    'normalized_num_leds': 200,
    'mcast_group': "239.1.2.2",
    "logical_device_id": -1,
    "idle_states": [
        {"action": "effect", "args": ["sethsv", "0", "1", ".3"]},
        {"action": "effect", "args": ["sethsv", "120", "1", ".3"]},
        {"action": "effect", "args": ["sethsv", "240", "1", ".3"]},
        {"action": "effect", "args": ["sethsv", "58", "1", ".3"]},
    ]
}

MAX_TOTAL_P = 45000
MCAST_PORT = 4545
INTRA_MCAST_PORT = 4546
led_run = True
abort_flag_lock = _thread.allocate_lock()
abort_flag = False
pending_command_lock = _thread.allocate_lock()
pending_commands = []
intra_command_lock = _thread.allocate_lock()
intra_commands = []
out_command_lock = _thread.allocate_lock()
out_commands = []
run_lock = _thread.allocate_lock()
global_dimming = 255
idle_state = None

led_values = []

def get_config(name):
    return DEVICE_CONFIG.get(
        name,
        DEFAULT_DEVICE_CONFIG.get(name)
    )

def sparkle(sm, rate = 50, *args):
    MIN_VAL = 40
    MAX_VAL = 255
    STEP = int(rate)
    SAT = 0.5
    BASE_HUE = 0
    ACCENT_HUE = 30
    denominator = 255
    prev_dirs = [False] * get_config('real_num_leds')
    base_color = HSV2RGB(BASE_HUE, SAT, 0 / denominator)
    weight = 0
    while not abort_flag:
        weight += 1
        dirs = [random.choice([True, True] + ([False] * weight)) for i in range(get_config('real_num_leds'))]
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
            for i in range(get_config('real_num_leds')):
                if dirs[i]:
                    pixels_set(i, up_color)
                elif prev_dirs[i]:
                    pixels_set(i, down_color)
                else:
                    pixels_set(i, base_color)
            pixels_show(sm)
        prev_dirs = dirs
    pixels_fill((0,0,0))
    pixels_show(sm)

def fade(sm, *args):
    hue = int(args[0])
    sat = float(args[1])
    start_val = float(args[2])
    end_val = float(args[3])
    iters = int(args[4])

    step = (end_val-start_val)/iters

    val = start_val
    i = iters
    while not abort_flag and (i >= 0):
        (r,g,b) = HSV2RGB(hue, sat, val)
        pixels_fill((r,g,b))
        pixels_show(sm)
        i -= 1
        val += step
        # Fudged value until we get about 50ms per iteration
        time.sleep_ms(30)

def chase(sm, *args):
    # HUE = 30
    HUE = random.randint(0, 359)
    SAT = .5
    MIN_VAL = 40
    MAX_VAL = 255
    STEP = 10
    pixels_fill((0,0,0))
    pixels_show(sm)
    precomputed_hue = [(0,0,0)]
    for current_bright in range(MIN_VAL, MAX_VAL, STEP):
        precomputed_hue.append(HSV2RGB(HUE, SAT, current_bright / 255))
    for led_i in range(get_config('real_num_leds')+len(precomputed_hue)-1):
        if abort_flag:
            return
        up_color = precomputed_hue[-1]
        pixels_set(led_i, up_color)
        if led_i > 1:
            trailing_hue = 2
            for led_reverse_i in (range(led_i-1, -1, -1)):
                try:
                    pixels_set(led_reverse_i, precomputed_hue[-trailing_hue])
                except IndexError:
                    break
                trailing_hue += 1
        pixels_show(sm)

def continuous_chase(sm, *args):
    while not abort_flag:
        chase(sm)
        if abort_flag:
            return
        time_to_wait = random.randint(500, 1500)
        while time_to_wait > 0:
            if abort_flag:
                break
            time.sleep_ms(10)
            time_to_wait -= 10

def candycane_wreath(sm, *args):
    pixels_fill((0,0,0))
    pixels_show(sm)
    interval = 6
    color_selector = 0
    hsv_choices = [(0, 1, .3), (0, 0, .3)]
    for led_i in range(get_config('real_num_leds')):
        if (led_i % interval) ==  0:
            color_selector ^= 1
        pixels_set(led_i, HSV2RGB(*hsv_choices[color_selector]))
    pixels_show(sm)


# def candycane_sphere(sm, *args):
#     pixels_fill((0,0,0))
#     pixels_show(sm)
#     s_step = 1/20
#     s_vals = [s_step*i for i in range(21)]
#     rgb_vals = [HSV2RGB(0, s, .3) for s in s_vals]
#     while not abort_flag:
#         for offset in range(len(rgb_vals)):
#             if abort_flag:
#                 break
#             for led_i in range(get_config('real_num_leds')):
#                 pixels_set(led_i, rgb_vals[(led_i+offset)%len(rgb_vals)])
#             pixels_show(sm)
#             time.sleep_ms(100)

def candycane_sphere(sm, *args):
    pixels_fill((0,0,0))
    pixels_show(sm)
    s_step = 1/2
    s_vals = [s_step*i for i in range(3)]
    rgb_vals = [HSV2RGB(0, s, .3) for s in s_vals] + [HSV2RGB(0, s, .3) for s in reversed(s_vals)] + ([HSV2RGB(0, s_vals[0], .3)] * 3)
    while not abort_flag:
        for offset in range(len(rgb_vals)):
            if abort_flag:
                break
            for led_i in range(get_config('real_num_leds')):
                pixels_set(led_i, rgb_vals[(led_i+offset)%len(rgb_vals)])
            pixels_show(sm)
            time.sleep_ms(100)

def global_fade_out(sm, rate, *args):
    global global_dimming
    save_global_dimming = global_dimming

    for d in range(global_dimming, 0, -int(rate)):
        global_dimming = d
        pixels_show(sm)

    pixels_fill((0,0,0))
    pixels_show(sm)
    global_dimming = save_global_dimming

def wreath_render_wand_point(sm, *args):
    # x = int(args[0])
    # y = int(args[1])
    # if (y < 150):
    #     color = HSV2RGB(120, 1, .4)
    # else:
    #     color = HSV2RGB(0, 1, .4)

    # for led_i in range(get_config('real_num_leds')):
    #     pixels_set(led_i, color)
    # pixels_show(sm)

    for led_i in range(get_config('real_num_leds')):
        pixels_set(led_i, HSV2RGB(60, 1, .3))
    pixels_show(sm)

def sethsv(sm, *args):
    h = int(args[0])
    (s,v) = [float(i) for i in args[1:3]]
    (r, g, b) = HSV2RGB(h, s, v)
    pixels_fill((r,g,b))
    pixels_show(sm)

def fade_sat(sm, hsv, reverse=False, invert=False):
    step_count = int((20 * (get_config('normalized_num_leds')/get_config('real_num_leds'))))
    if abort_flag:
        return
    h, s, v = hsv
    s_step = s/step_count
    if reverse:
        s_vals = reversed([s_step*i for i in range(step_count+1)])
    else:
        s_vals = [s_step*i for i in range(step_count+1)]
    if invert:
        shades = [HSV2RGB(h, 1-s_val, v) for s_val in s_vals]
    else:
        shades = [HSV2RGB(h, s_val, v) for s_val in s_vals]
    for shade in shades:
        if abort_flag:
            break
        pixels_fill(shade)
        pixels_show(sm)

def fade_val(sm, hsv, reverse=False):
    step_count = int((20 * (get_config('normalized_num_leds')/get_config('real_num_leds'))))
    if abort_flag:
        return
    h, s, v = hsv
    v_step = v/step_count
    if reverse:
        v_vals = reversed([v_step*i for i in range(step_count+1)])
    else:
        v_vals = [v_step*i for i in range(step_count+1)]
    shades = [HSV2RGB(h, s, v_val) for v_val in v_vals]
    for shade in shades:
        if abort_flag:
            break
        pixels_fill(shade)
        pixels_show(sm)

def christmas_color_cycle(sm):
    time.sleep_ms(random.randint(500,2500))
    colors = [
        (0, 1, .3),
        (120, 1, .3),
        (240, 1, .3),
        (58, 1, .3),
        (29, 1, .3),
    ]
    while not abort_flag:
        color = random.choice(colors)
        if abort_flag:
            break
        fade_sat(sm, color, reverse=False)
        waited = 0
        wait_per_iter = 500
        while waited < 5000:
            if abort_flag:
                break
            time.sleep_ms(wait_per_iter)
            waited += wait_per_iter
        fade_sat(sm, color, reverse=True)

def christmas_color_cycle2(sm):
    time.sleep_ms(random.randint(500,2500))
    colors = [
        (0, .5, .3),
        (120, .5, .3),
        (240, .5, .3),
        (58, .5, .3),
        (29, .5, .3),
    ]
    while not abort_flag:
        color = random.choice(colors)
        if abort_flag:
            break
        fade_sat(sm, color, reverse=False, invert=True)
        waited = 0
        wait_per_iter = 500
        while waited < 3000:
            if abort_flag:
                break
            time.sleep_ms(wait_per_iter)
            waited += wait_per_iter
        fade_sat(sm, color, reverse=True, invert=True)

def hot_potato(sm, max_logical_id, expanded_potato_colors=False):
    clear_intra_commands()
    my_id = get_config('logical_device_id')
    if not expanded_potato_colors:
        potato_colors = [(0, 1, .3)]
    else:
        potato_colors = [
            (0, 1, .3),
            (120, 1, .3),
            (240, 1, .3),
            (58, 1, .3),
            (29, 1, .3),
        ]
    msg_id = 0

    if my_id == 0:
        is_head = True
    else:
        is_head = False
    if is_head:
        has_potato = True
    else:
        pixels_fill(HSV2RGB(0, 0, .3))
        pixels_show(sm)
        has_potato = False

    color_idx = random.randint(0, len(potato_colors)-1)
    while not abort_flag:
        if has_potato:
            if my_id == int(max_logical_id):
                next_id = 0
            else:
                next_id = my_id + 1
            msg_id = time.time_ns()
            msg_data = {
                'baton': 'hot_potato',
                'id': msg_id,
                'dest': [next_id],
                'color': (color_idx+1) % len(potato_colors)
            }
            push_out_command(msg_data)
            has_potato = False
            fade_sat(sm, potato_colors[color_idx], reverse=True)

        cmd = pop_intra_command()
        if not cmd:
            if (time.time_ns() - msg_id) > 8000000000 and is_head:
                has_potato=True
            continue
        if cmd.get('baton') != 'hot_potato':
            print("unknown baton.  dropping!")
        has_potato = True
        if is_head:
            color_idx = random.randint(0, len(potato_colors)-1)
        else:
            color_idx = cmd.get('color')
        fade_sat(sm, potato_colors[color_idx], reverse=False)

def naughty_or_nice(sm):
    clear_intra_commands()
    my_id = get_config('logical_device_id')
    naughty_color = (0, 1, .3)
    nice_color = (120, 1, .3)
    thinking_color = (50, 1, .3)
    msg_id = 0

    if my_id == 0:
        is_head = True
    else:
        is_head = False

    reverse = False
    time.sleep_ms(random.randint(200,700))
    for _ in range(6):
        if abort_flag:
            return
        fade_val(sm, thinking_color, reverse=reverse)
        reverse = not reverse

    if is_head:
        is_naughty = random.choice([True] + [False] * 9)
        msg_id = time.time_ns()
        msg_data = {
            'baton': 'naughty_or_nice',
            'is_naughty': is_naughty,
            'id': msg_id
        }
        push_out_command(msg_data)

        if is_naughty:
            fade_val(sm, naughty_color, reverse=False)
        else:
            fade_val(sm, nice_color, reverse=False)
    else:
        while not abort_flag:
            cmd = pop_intra_command()
            if not cmd:
                continue
            if cmd.get('baton') != 'naughty_or_nice':
                print("unknown baton.  dropping!")
            else:
                is_naughty = cmd.get('is_naughty')
                if is_naughty:
                    fade_val(sm, naughty_color, reverse=False)
                    break
                else:
                    fade_val(sm, nice_color, reverse=False)
                    break

ROUTINES = {
    'sparkle': sparkle,
    'fade': fade,
    'chase': chase,
    'continuous_chase': continuous_chase,
    'candycane_wreath': candycane_wreath,
    'candycane_sphere': candycane_sphere,
    'global_fade_out': global_fade_out,
    'wreath_render_wand_point': wreath_render_wand_point,
    'sethsv': sethsv,
    'christmas_color_cycle': christmas_color_cycle,
    'christmas_color_cycle2': christmas_color_cycle2,
    'hot_potato': hot_potato
}

def pixels_show(sm, dim_val = None):
    if dim_val is None:
        set_dim_val = global_dimming
    else:
        set_dim_val = dim_val
    leds_packed = array.array("I", [0 for _ in range(get_config('real_num_leds'))])
    total_p = 0
    for i, pixel in enumerate(led_values):
        adj_red = ((pixel[0] ** 2) * set_dim_val) // 255**2
        adj_green = ((pixel[1] ** 2) * set_dim_val) // 255**2
        adj_blue = ((pixel[2] ** 2) * set_dim_val) // 255**2
        total_p += adj_red + adj_green + adj_blue
        leds_packed[i] = (adj_blue << 16) + (adj_red << 8) + adj_green
    adj_max_total_p = (MAX_TOTAL_P * get_config('real_num_leds')) // get_config('normalized_num_leds')
    if total_p > adj_max_total_p:
        new_dim_val = (global_dimming*adj_max_total_p) // total_p
        # The -1000 is a hack here because we sometimes seem to calculate
        # brightness values so they are never less than adj_max_total_p
        pixels_show(sm, new_dim_val-1000)
        return
    sm.put(leds_packed, 8)
    time.sleep_us(100)


def pixels_set(i, val):
    # Allow overflow to make trailing effects easier
    if i >= len(led_values):
        return
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

def push_intra_command(command):
    intra_command_lock.acquire()
    intra_commands.insert(0, command)
    intra_command_lock.release()

def pop_intra_command():
    intra_command_lock.acquire()
    if len(intra_commands) == 0:
        ps = None
    else:
        ps = intra_commands.pop()
    intra_command_lock.release()
    return ps

def clear_intra_commands(abort = False):
    intra_command_lock.acquire()
    intra_commands.clear()
    if abort:
        abort_current_command()
    intra_command_lock.release()

def push_out_command(command):
    out_command_lock.acquire()
    out_commands.insert(0, command)
    out_command_lock.release()

def pop_out_command():
    out_command_lock.acquire()
    if len(out_commands) == 0:
        ps = None
    else:
        ps = out_commands.pop()
    out_command_lock.release()
    return ps

def led_state_thread(sm):
    print("Starting")
    while led_run:
        command = pop_pending_command()
        if command is None:
            continue
        clear_abort_flag()
        try:
            run_lock.acquire()
            action = command['action']
            args = command.get('args', [])
            if action == 'effect':
                routine = args[0]
                if routine in ROUTINES:
                    func = ROUTINES[routine]
                    if len(args) > 1:
                        func(sm, *args[1:])
                    else:
                        func(sm)
                else:
                    print(f"effect: unknown function: {routine}")
            elif action == 'spell':
                if args[0] == 'incendio':
                    if get_config("description") == "wreath":
                        sethsv(sm, 0, 0, .2)
                    else:
                        sparkle(sm, 200)
                if args[0] == 'mimblewimble':
                    if get_config("description") == "wreath":
                        sethsv(sm, 0, 0, .2)
                    else:
                        naughty_or_nice(sm)
                if args[0] == 'gonadium':
                    fade_sat(sm, (240, 1.0, .4), reverse=False)
            elif action == 'training_enter':
                fade(sm, 50, .4, 0, .2, 10)
            else:
                print(f"Unknown action: {action}")
        except Exception as e:
            print(f"Bad command: {command}: {e}")
            sys.print_exception(e)
        finally:
            run_lock.release()

    print("Exiting")

def download_new_main(expected_sha1):
    import urequests
    import os
    import hashlib
    import binascii

    MAIN_PY_URL = f"{CONTROLLER_BASE_URL}/main.py"
    print("Attempting main.py update")
    current_sha1 = hashlib.sha1()
    with open("/main.py", "rb") as f:
        current_sha1.update(f.read())
    current_hexdigest = binascii.hexlify(current_sha1.digest()).decode()
    if current_hexdigest == expected_sha1:
        # Alread has this file, nothing to do.
        print("main.py already matches hash")
        return
    print(f"Checking for new main.py at {MAIN_PY_URL}")
    try:
        r = urequests.get(MAIN_PY_URL, timeout=5.0)
        if r.status_code != 200:
            raise Exception(f"Bad http response: {r.status_code}")
        main_py_source = r.text
    except Exception as e:
        print(f"Download Failed: {e}")
        return

    with open("/main.py.download", "w") as f:
        f.write(main_py_source)

    new_sha1 = hashlib.sha1()
    with open("/main.py.download", "rb") as f:
        new_sha1.update(f.read())
    new_hexdigest = binascii.hexlify(new_sha1.digest()).decode()
    if new_hexdigest != expected_sha1:
        print("sha1 mismatch!")
        return

    os.unlink("/main.py")
    os.rename("/main.py.download", "/main.py")
    machine.reset()

def fetch_device_config():
    try:
        wlan_sta = network.WLAN(network.STA_IF)
        wlan_mac = wlan_sta.config('mac')
        my_mac = ubinascii.hexlify(wlan_mac).decode()
        print(f"my_mac: {my_mac}")
        device_config_url = f"{CONTROLLER_BASE_URL}/device_configs/{my_mac}"
    except Exception as e:
        print(f"Failed discovering MAC address: {e}")

    try:
        r = urequests.get(device_config_url, timeout=5.0)
        if r.status_code != 200:
            raise Exception(f"Bad http response: {r.status_code}")
        config_text = r.text
        print("Device config download OK.  Saving file.")
        with open("/device_config.json", "w") as f:
            f.write(config_text)
    except Exception as e:
        print(f"Config fetch Failed: {e}")

    try:
        print("Trying to load saved config")
        with open("/device_config.json") as f:
            config_data = json.load(f)

        global DEVICE_CONFIG
        DEVICE_CONFIG = config_data
    except Exception as e:
        print(f"Loading saved config failed.  Using defaults! {e}")


def handle_cmd(sm, command):
    global idle_state
    global last_msg_id
    try:
        action = command['action']
        if action != "idlestate" and idle_state is not None:
            idle_state = None
            print("end idlestate")
            clear_pending_commands(abort=True)
            global_fade_out(sm, 20)
            print("idlestate ended")
        if action == 'abort':
            clear_pending_commands(abort=True)
            if command.get('blank', False):
                run_lock.acquire()
                pixels_fill((0, 0, 0))
                pixels_show(sm)
                run_lock.release()
        elif action == 'reboot':
            machine.reset()
        elif action == 'update_main':
            new_sha1 = command['sha1']
            download_new_main(expected_sha1=new_sha1)
        elif action == 'idlestate':
            idle_state_arg = command.get('args', [0])[0]
            if idle_state is None or idle_state != idle_state_arg:
                idle_state = idle_state_arg
                idle_effect = get_config('idle_states')[idle_state % len(get_config('idle_states'))]
                clear_pending_commands(abort=True)
                global_fade_out(sm, 20)
                push_pending_command(idle_effect)
        else:
            push_pending_command(command)
    except Exception as e:
        print(f"Error processing command: {e}")


def main(sm, ip):
    global led_run
    pixels_fill((0, 0, 0))
    pixels_show(sm)

    fetch_device_config()
    global led_values
    led_values = [(0,0,0)] * get_config('real_num_leds')

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    cmd_sock.setsockopt(
        socket.IPPROTO_IP,
        socket.IP_ADD_MEMBERSHIP,
        inet_aton(get_config('mcast_group')) + inet_aton(ip),
    )
    cmd_sock.bind(("", MCAST_PORT))

    intra_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    intra_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    intra_sock.setsockopt(
        socket.IPPROTO_IP,
        socket.IP_ADD_MEMBERSHIP,
        inet_aton(get_config('mcast_group')) + inet_aton(ip),
    )
    intra_sock.bind(("", INTRA_MCAST_PORT))

    poller = select.poll()
    poller.register(cmd_sock, select.POLLIN)
    poller.register(intra_sock, select.POLLIN)

    _thread.start_new_thread(led_state_thread, [sm])

    last_msg_id = {
        cmd_sock: -1,
        intra_sock: -1
    }

    try:
        while True:
            res = poller.poll(50)
            if res:
                for s in res:
                    data, _ = s[0].recvfrom(256)
                    command = json.loads(data.decode())
                    msg_id = command.get('id')
                    dest = command.get('dest')
                    if type(dest) is not list:
                        dest = [dest]
                    if dest[0] is not None and get_config('logical_device_id') not in dest:
                        continue
                    if msg_id is not None and msg_id == last_msg_id[s[0]]:
                        continue
                    last_msg_id[s[0]] = msg_id
                    if s[0] is intra_sock:
                        push_intra_command(command)
                    else:
                        handle_cmd(sm, command)
            else:
                out_cmd = pop_out_command()
                out_cmd_json = json.dumps(out_cmd)
                if out_cmd:
                    intra_sock.sendto(out_cmd_json, (get_config('mcast_group'), INTRA_MCAST_PORT))
                    time.sleep_ms(100)
                    intra_sock.sendto(out_cmd_json, (get_config('mcast_group'), INTRA_MCAST_PORT))
                    time.sleep_ms(100)
                    intra_sock.sendto(out_cmd_json, (get_config('mcast_group'), INTRA_MCAST_PORT))
    finally:
        cmd_sock.close()
        intra_sock.close()
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
