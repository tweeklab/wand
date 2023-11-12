# Example using PIO to drive a set of WS2812 LEDs.

import array, time
import os
from machine import Pin
import rp2
import network
import urequests

FW_URL = "http://10.0.0.30:4545/globe_firmware.py"
WIFI_SSID = ''
WIFI_PASSWORD = ''
NUM_LEDS = 200
PIN_NUM = 22

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()

# Create the StateMachine with the ws2812 program, outputting on pin
sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(PIN_NUM))

# Start the StateMachine, it will wait for data on its FIFO.
sm.active(1)

# Display a pattern on the LEDs via an array of LED RGB values.
led_values = array.array("I", [0 for _ in range(NUM_LEDS)])

INIT_MAIN_EXIT = (5, 0, 0)
INIT_WIFI_CONNECTING = (0, 0, 20)
INIT_WIFI_CONNECTED = (0, 20, 20)
INIT_FW_UPDATING = (20, 20, 0)
INIT_FW_UPDATED = (20, 20, 20)
INIT_FW_UPDATE_FAIL = (20, 0, 20)
INIT_FW_LOADED = (0, 20, 0)
INIT_FW_LOAD_FAIL = (20, 0, 0)

##########################################################################
def set_led_status(status):
    for i in range(len(led_values)):
        led_values[i] = (status[2]<<16) + (status[0]<<8) + status[1]
    sm.put(led_values, 8)
    time.sleep_ms(10)

def connect():
    set_led_status(INIT_WIFI_CONNECTING)
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    while wlan.isconnected() == False:
        print('Waiting for connection...')
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    set_led_status(INIT_WIFI_CONNECTED)
    return ip

def fw_update():
    print(f"Checking for firmware at {FW_URL}")
    try:
        r = urequests.get(FW_URL, timeout=5.0)
        if r.status_code != 200:
            raise Exception(f"Bad http response: {r.status_code}")
        fw_source = r.text
    except Exception as e:
        print(f"Download Failed: {e}")
        return None

    with open("/firmware_tmp.py", "w") as f:
        f.write(fw_source)
    try:
        from firmware_tmp import main as new_fw_main
    except Exception as e:
        print(f"Firmware load failed: {e}")
        os.unlink('/firmware_tmp.py')
        return None

    try:
        os.unlink('/firmware.py')
    except:
        pass
    os.rename('/firmware_tmp.py', 'firmware.py')

    return new_fw_main

def fw_load():
    set_led_status(INIT_FW_UPDATING)
    new_fw_main = fw_update()
    if new_fw_main is not None:
        set_led_status(INIT_FW_UPDATED)
        return new_fw_main
    else:
        set_led_status(INIT_FW_UPDATE_FAIL)
        print("New firmware unavailable, try local")

    try:
        from firmware import main as fw_main
    except Exception as e:
        print(f"Existing firmware load failed: {e}")
        return None

    return fw_main

def bootloader():
    time.sleep(1)
    ip = connect()

    fw_main = fw_load()
    if fw_main:
        set_led_status(INIT_FW_LOADED)
        try:
            fw_main(sm, ip)
        except Exception as e:
            print(f"main raised exception: {e}")
        set_led_status(INIT_MAIN_EXIT)
    else:
        set_led_status(INIT_FW_LOAD_FAIL)

if __name__ == '__main__':
    bootloader()