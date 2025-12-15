"""Simple PWM test for ESC throttle on GPIO pin 18."""

import time

try:
    import RPi.GPIO as GPIO
except ModuleNotFoundError as exc:
    raise SystemExit("Install RPi.GPIO (e.g. 'pip install RPi.GPIO' or 'pip install rpi-lgpio')") from exc

GPIO.setmode(GPIO.BCM)
esc_pin = 18
GPIO.setup(esc_pin, GPIO.OUT)

pwm = GPIO.PWM(esc_pin, 50)
pwm.start(5)

time.sleep(0.5)

def set_speed(duty):
    pwm.ChangeDutyCycle(duty)

try:
    print("Arming ESC...")
    set_speed(5)   # minimum throttle
    time.sleep(2)

    print("Half throttle")
    set_speed(7.5)
    time.sleep(5)

    print("Max throttle")
    set_speed(10)
    time.sleep(5)

    print("Stopping motor")
    set_speed(5)   # send minimum throttle before exit
    time.sleep(2)

except KeyboardInterrupt:
    print("\nKeyboardInterrupt received, returning to minimum throttle...")
    set_speed(5)
    time.sleep(2)

finally:
    pwm.stop()
    # Explicitly delete PWM object before cleanup so its destructor does not
    # run after the lgpio handle has been torn down.
    del pwm
    GPIO.cleanup()
    print("GPIO cleanup done, motor stopped")