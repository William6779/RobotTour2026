#!/usr/bin/env python3

# to kill daemon run this: sudo systemctl stop gpio-buttons.service
#
import subprocess, time, os
from gpiozero import Button


# ---------- CONFIG ----------
# BCM pin numbers (you said START=10, RESET=9, SHUTDOWN=11)
PIN_START    = 9


# Require a short hold to trigger (avoids accidental brushes)
HOLD_SEC   = 0.1
BOUNCE_SEC = 0.05

# The shell script to launch on START:
PROGRAM_CMD = ["/usr/bin/bash", "/home/pi/projects/rt/start_app.sh"]  # <- change path if needed
# ----------------------------

child = None          # track started process so we don't launch twice

def do_start():
    """Start script once; ignore if it's already running."""
    global child
    if child is not None and child.poll() is None:
        # already running
        return
    # start detached so it keeps running independently
    child = subprocess.Popen(PROGRAM_CMD, start_new_session=True)


# Buttons use internal pull-ups; wire each button to its GPIO and GND
btn_start    = Button(PIN_START,    pull_up=True, bounce_time=BOUNCE_SEC, hold_time=HOLD_SEC)


# Fire only after they are held for HOLD_SEC
btn_start.when_held    = do_start


# Keep process alive
print("GPIO buttons active. Hold START for 1s to trigger. Ctrl+C to exit.")
try:

    while True:
        if child is not None:
            if child.poll() is not None:
                # process ended
                child = None
        time.sleep(1)


except KeyboardInterrupt:
    pass
