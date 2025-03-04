# PID Demo (Template) | v1.1
# Authored by Colin Wong
# Requirements: pygame, matplotlib

# This file serves as a demo template to implement a PID controller. The target of
# PID control is a red circle that follows your cursor around the window. To fill
# out the template, implement the lines labeled TODO in PIDController's calculate
# method. Then, tune the PID controllers you implemented by changing the arguments
# to the PIDController constructors for the X and Y error controllers underneath
# the PIDController class definition. 

import sys, pygame, math, time
from pygame import draw, mouse, Vector2
import matplotlib.animation as anim
import matplotlib.pyplot as plt
from time import sleep
from collections import deque
pygame.init()

def clamp(val: float, min_val: float, max_val: float):
    return max(min_val, min(max_val, val))

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        # PID constants
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Saved state for I/D terms
        self._accum = 0
        self._prev_error = 0

        # Saved state for velocity tolerance
        self._prev_diff = 0

        # Tolerances for at_setpoint
        self._pos_tolerance = 0.1
        self._vel_tolerance = 0.05

        # Minimum and maximum accumulator values
        self._minmax_accum = (-200, 200)

    def calculate(self, proc_var: float, setpoint: float):
        # Calculate error (TODO)
        error = 0

        # Calculate P value (TODO)
        val_p = 0

        # Reset accumulator if error crosses 0
        if math.copysign(1, self._prev_error) != math.copysign(1, error):
            self._accum = 0
        # Accumulate value (TODO)
        # Clamp accumulator between min and max values (TODO)      
        # Calculate I value (TODO)  
        val_i = 0

        # Calculate error delta (TODO)
        diff = 0
        # Calculate D value (TODO)
        val_d = 0

        # Set stateful variables (TODO)

        # Return sum of PID terms
        return val_p + val_i + val_d

    def reset(self):
        self._accum = 0
        self._prev_error = 0
        self._prev_diff = 0

    def at_setpoint(self):
        return abs(self._prev_error) <= self._pos_tolerance and abs(self._prev_diff) <= self._vel_tolerance

# TODO change PID constants
pidx = PIDController(1, 0, 0)
pidy = PIDController(1, 0, 0)


size = width, height = 640, 640
max_accel = 3
background_color = 200, 200, 255
circle_color = 255, 0, 0

screen = pygame.display.set_mode(size)

pos: Vector2 = Vector2(0, 0)
target_pos = Vector2(mouse.get_pos())

fig, ax = plt.subplots(1, 1)

ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)

window_vals = [ (n*0.02) - 2.5 for n in range(251) ]
x_err: deque[float] = deque([0]*251, maxlen=251)
y_err: deque[float] = deque([0]*251, maxlen=251)

y_line, x_line = ax.plot(window_vals, y_err, 'b-', x_err, window_vals, 'r-')


def update_plt(frame):
    x_line.set_data(x_err, window_vals)
    y_line.set_data(window_vals, y_err)
    return x_line, y_line

ani = anim.FuncAnimation(fig, update_plt, frames=len(x_err), interval=20, blit=True)

plt.show(block=False)

prev_outx = 0
prev_outy = 0

last_time = -1

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()
        if event.type == pygame.MOUSEBUTTONDOWN:
            last_time = time.time_ns()

    outx = pidx.calculate(pos.x, target_pos.x)
    outy = pidy.calculate(pos.y, target_pos.y)

    outx = clamp(outx, prev_outx - max_accel, prev_outx + max_accel)
    outy = clamp(outy, prev_outy - max_accel, prev_outy + max_accel)

    pos.x += outx
    pos.y += outy

    prev_outx = outx
    prev_outy = outy

    x_err.append(-pidx._prev_error / width * 2.5)
    y_err.append(pidy._prev_error / height * 2.5)

    if pidx.at_setpoint():
        pidx.reset()
    if pidy.at_setpoint():
        pidy.reset()
    if last_time > 0 and pidx.at_setpoint() and pidy.at_setpoint():
        t_diff = (time.time_ns() - last_time) / (10**9)
        if t_diff > 0.3:
            print(f'Time elapsed: {t_diff}')
            last_time = -1

    screen.fill(background_color)
    draw.circle(screen, circle_color, pos, 20)
    pygame.display.flip()

    target_pos = Vector2(mouse.get_pos())

    sleep(0.02)
