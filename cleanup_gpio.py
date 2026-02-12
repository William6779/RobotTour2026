import RPi.GPIO as GPIO
GPIO.cleanup()
try:
    from gpiozero import Device
    Device.pin_factory.release_all()
except Exception:
    pass
