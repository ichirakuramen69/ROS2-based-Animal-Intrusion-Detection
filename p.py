import RPi.GPIO as GPIO, time
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
p = GPIO.PWM(18, 50)
p.start(7.5)   # 1500us center
time.sleep(1)
def angle_to_duty(a):
    pulse = 500 + (a/180.0)*(2500-500)
    return pulse/200.0
for a in (0, 45, 90, 135, 180):
    p.ChangeDutyCycle(angle_to_duty(a))
    time.sleep(0.8)
p.ChangeDutyCycle(0)
p.stop()
GPIO.cleanup()
