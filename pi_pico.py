from machine import Pin,ADC,PWM
import time
from dht11 import DHT11
import utime
from ultrasonic import ultrasonic


servo = PWM(Pin(7))
servo.freq(50)
light = ADC(28)
dth = Pin(22, Pin.OUT)
Echo = Pin(13, Pin.IN)
Trig = Pin(14, Pin.OUT)
in1 = Pin(16, Pin.OUT)
in2 = Pin(15, Pin.OUT)
in3 = Pin(14, Pin.OUT)
in4 = Pin(13, Pin.OUT)

ultrasonic = ultrasonic(Trig, Echo)
dht11 = DHT11(dth)
delay = 1

ROUND_VALUE = 509

STEP_VALUE = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1],
]

def reset():
    in1(0)
    in2(0)
    in3(0)
    in4(0)

def step_run(count):
    direction = 1     # turn clockwise
    if count < 0:
        direction = -1  # turn counterclockwise
        count = -count
    for x in range(count):
        for bit in STEP_VALUE[::direction]:
            in1(bit[0])
            in2(bit[1])
            in3(bit[2])
            in4(bit[3])
            utime.sleep_ms(delay)
    reset()

def step_angle(a):
    step_run(int(ROUND_VALUE * a / 360))

def my_map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def get_value():
    return int(light.read_u16() * 101 / 65536)

def servo_control(value):
    if value > 180 or value < 0:
        print('Please enter a limited speed value of 0-180 ')
        return
    duty = my_map(value, 0, 180, 500000, 2500000)
    servo.duty_ns(duty)

while True:
    servo_control(0)
    utime.sleep(1)
    servo_control(180)
    utime.sleep(1)
    value = get_value()
    print(get_value())
    utime.sleep(.1)
    distance = ultrasonic.Distance_accurate()
    print("distance is %d cm"%(distance))
    utime.sleep(.5)
    print("temperature is %d ℃" % dht11.temperature)
    time.sleep(.5)
    print("humidity is %d " % dht11.humidity)
    time.sleep(.5)
    step_run(509)
    step_run(-509)
    step_angle(360)
    step_angle(-360)