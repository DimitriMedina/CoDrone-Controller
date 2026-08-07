from pynput import keyboard
from pynput.keyboard import Key

stop_flag = False


def on_press(key):
    global stop_flag
    try:
        if key.char == "q":
            stop_flag = True
    except AttributeError:
        stop_flag = False


def listen_for_keys():
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
