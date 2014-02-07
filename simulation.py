# COMAP Contest
#
# Highway Traffic Simulation
# Two-Lane, IDM Model
# SI units

import numpy as np
import math

DT = 0.1        # s
V0 = 29.0575    # m/s - nominal desired speed
S0 = 1.2        # m - minimum spacing
TH = 2.4        # s - desired time headway
A = 2.78        # m/s^2 - nominal acceleration

class Vehicle:

    l = 6       # m - nominal vehicle length

    v0 = V0     # desired speed
    th = TH     # desired time headway
    a = A       # acceleration

    x0 = 0      # initial displacement
    y0 = 0      # initial lateral displacement
    dx0 = v0    # initial speed

    def __init__(self):
        pass

vehicle1 = Vehicle()
vehicle1.x = 0

vehicle2 = Vehicle()
vehicle2.x = 8

vehicle3 = Vehicle()
vehicle3.x = 18

vehicles  = [vehicle1, vehicle2, vehicle3]


t_start = 0
t_end = 30


N = len(vehicles)
x = np.zeros(1,N)
dx = np.zeros(1,N)
# initialize vehicle positions and velocities
for i in range(N):
    x[i] = vehicles[i].x0
    dx[i] = vehicles[i].dx0

def simulate():
    while t_start < t_end:




def main():
    print("hello")


if __name__ == "__main__":
    main()
