# -----------------------
#       Detección
# -----------------------

import sensor
import time
import pyb
from machine import LED, Pin

pyb.LED(3).on()

# Declaración de variables para cámara

THRESHOLD = (100, 200)  # Umbral de escala de grsis para objetos oscuros
BINARY_VISIBLE = True  # Paso binario primero para ver en qué se está ejecutando la regresión lineal.

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)
sensor.skip_frames(time=2000)
clock = time.clock()

# -----------------------
#      Comunicación
# -----------------------

from pyb import Pin, Timer

p = Pin('D1') # X1 has TIM2, CH1
tim = Timer(1, freq=10000000)
ch = tim.channel(2, Timer.PWM, pin=p)
ch.pulse_width_percent(0)

while True:

  clock.tick()

  img = sensor.snapshot().binary([THRESHOLD],invert=True) if BINARY_VISIBLE else sensor.snapshot() #Activación de la cámara con la activación condicionada del filtro binario.
  line = img.get_regression(
    [(255, 255) if BINARY_VISIBLE else THRESHOLD], robust=True
  )

  if line:

    img.draw_line(line.line(), color=127)

    theta = line.theta()
    if  theta < 90:
        angulo = theta + 2*(45-theta)
    else:
        angulo = theta + 2*(135-theta)

    print(angulo)
    pwm = 100*angulo/180
    ch.pulse_width_percent(int(pwm))
    print(pwm)
