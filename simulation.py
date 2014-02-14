# COMAP Contest
#
# Highway Traffic Simulation
# Two-Lane, IDM Model
# SI units

import math
import random
import itertools
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Vehicle:

    def __init__(self):
        self.x = 0.      # displacement
        self.lane = 0    # highway lane
        self.v = 30.     # speed
        self.v0 = 30.    # desired velocity
        self.th0 = 1.5   # desired time heading to next vehicle
        self.a = .3      # acceleration
        self.b = 3.      # comfortable braking decelleration
        self.s0 = 1.5    # minimum spacing
        self.l = 5.      # nominal vehicle length
        self.p = 0.5     # politeness factor
    
class TrafficGenModel:
    # traffic generation parameters
    # Cowen M4
    # http://www.uwstarlab.org/STARLab_Papers/2007_TRB_Examining%20Headway%20Distribution%20Models%20Using%20Urban%20Freeway%20Loop%20Event%20Data.pdf
    # delta = 1.8     # min headway (s)
    # phi = 0.9       # proportion of free vehicles
    # lamb = 0.4      # scale parameter

    def __init__(self, lane=0): 
        self.delta = 1.2
        self.phi = 5.8
        self.lamb = 0.9
        self.lane = lane
        random.seed()

    def next_headway_interval(self):
        x = random.uniform(0., 1.)
        t = -1./self.lamb * math.log((1. - x)/self.phi) + self.delta
        return t

    def next_vehicle(self):
        vehicle = Vehicle()
        vehicle.lane = self.lane
        vehicle.v0 = random.gauss(vehicle.v0, 4.)
        vehicle.th0 = vehicle.th0 + random.paretovariate(3.)
        vehicle.a = random.gauss(vehicle.a, 0.05)
        vehicle.b = random.gauss(vehicle.b, 0.05)
        vehicle.p = random.gauss(0.5, 0.3)
        return vehicle


def gaussian(x, sigma, a):
    return a / (sigma * math.sqrt(2. * math.pi)) * math.exp(-math.pow(x/sigma, 2.) / 2.)

def idm_accel(vehicle1, vehicle2):
    a, b, x, v, v0, s0, th0 = vehicle1.a, vehicle1.b, vehicle1.x, vehicle1.v, \
                                vehicle1.v0, vehicle1.s0, vehicle1.th0
    x_next, l_next, v_next = vehicle2.x, vehicle2.l, vehicle2.v
    s = x_next - x - l_next
    dv = v - v_next
    s_star = s0 + v * th0 + v * dv / (2. * math.sqrt(a * b))
    return a * (1. - (v / v0)**4. - (s_star / s)**2.)

def comp_sigma_v(vehicles, x_start, x_stop, lane=-1):
    if lane == -1:
        x = [v.v - v.v0 for v in vehicles if v.x > x_start and v.x < x_stop]
        return sum(x) / (1. + len(x))
    else:
        x = [v.v - v.v0 for v in vehicles if v.lane == lane and v.x > x_start and v.x < x_stop]
        return sum(x) / (1. + len(x))



def comp_sigma_x(vehicles, x_start, x_stop, lane=-1):
    x = 0.
    for i in range(len(vehicles)):
        if lane == -1 or vehicles[i].lane == lane:
            x += next((v.x - vehicles[i].x for v in vehicles[i+1:] if v.lane == vehicles[i].lane), TrafficSimulation.dummy_ahead.x - vehicles[i].x)
    return x / (1. + len(vehicles))

def comp_safety_hist(vehicles, x_start, x_stop, hist_vals, lane=-1):
    vals = [0.] * len(hist_vals)
    for i in range(len(vehicles)):
        if lane == -1 or vehicles[i].lane == lane:
            t = next(((2.*(abs(v.x - vehicles[i].x))/(v.v + vehicles[i].v)) for v in vehicles[i+1:] if v.lane == vehicles[i].lane), TrafficSimulation.dummy_ahead.x - vehicles[i].x)
            for h_ind in range(len(hist_vals)):
                if t < hist_vals[h_ind]:
                    vals[h_ind] += 1 
    return vals

        
 
class TrafficSimulation:

    dummy_ahead = Vehicle()
    dummy_behind = Vehicle()

    def __init__(self, n_lanes=2):
        self.n_lanes = n_lanes
        self.gen_models = [TrafficGenModel(lane=i) for i in range(self.n_lanes)]
        self.next_gen_times = [0.] * self.n_lanes
        self.vehicles = []
        self.t_start = 0.    # simulation start time 
        self.t_end = 20.     # simulation end time
        self.x_start = 0.    # simulation highway start x
        self.x_end = 5000.   # simulation highway end x
        self.h = 0.2         # step size
        # dummy vehicles
        #self.dummy_ahead = Vehicle()
        TrafficSimulation.dummy_ahead.x = self.x_end + 400.
        #self.dummy_behind = Vehicle()
        TrafficSimulation.dummy_behind.x = self.x_start - 1000.


    def step(self, i):
        n_lanes = self.n_lanes
        x_end = self.x_end
        x_start = self.x_start
        vehicles = self.vehicles
        h = self.h
        time = self.t_start + i * h

        # vehicle generation
        for lane in range(n_lanes):
            if time >= self.next_gen_times[lane]: # time for lane
                vehicles.insert(0, self.gen_models[lane].next_vehicle()) # vehicle in lane
                self.next_gen_times[lane] = time + self.gen_models[lane].next_headway_interval()

        # sort by x position
        vehicles.sort(key = lambda obj: obj.x)
        # move each vehicle
        i = 0
        while i < len(vehicles):
            vehicle = vehicles[i]
            front = next((v for v in vehicles[i+1:] if v.lane == vehicle.lane), TrafficSimulation.dummy_ahead)
            front_l = next((v for v in vehicles[i+1:] if v.lane == vehicle.lane + 1), TrafficSimulation.dummy_ahead)
            front_r = next((v for v in vehicles[i+1:] if v.lane == vehicle.lane - 1), TrafficSimulation.dummy_ahead)
            back = next((v for v in reversed(vehicles[:i]) if v.lane == vehicle.lane), TrafficSimulation.dummy_behind)
            back_l = next((v for v in reversed(vehicles[:i]) if v.lane == vehicle.lane + 1), TrafficSimulation.dummy_behind)
            back_r = next((v for v in reversed(vehicles[:i]) if v.lane == vehicle.lane - 1), TrafficSimulation.dummy_behind)
            # if vehicle not in front, place at a distance
            # if vehicle not behind, place far behind
            # perform integration
            # from Wikipedia page: http://en.wikipedia.org/wiki/Intelligent_driver_model
            # potential acceleration in middle, left, right
            m_acc_c = idm_accel(vehicle, front)
            m_acc_l = idm_accel(vehicle, front_l)
            m_acc_r = idm_accel(vehicle, front_r)
            
            b_acc_c_1 = idm_accel(back, vehicle)
            b_acc_c_2 = idm_accel(back, front)
            b_acc_l_1 = idm_accel(back_l, front_l)
            b_acc_l_2 = idm_accel(back_l, vehicle)
            b_acc_r_1 = idm_accel(back_r, front_r)
            b_acc_r_2 = idm_accel(back_r, vehicle)

            # use MOBIL model for lane changing
            b_safe = -4.0
            thresh_l = 0.2
            thresh_r = -0.05
            a_bias = 0.2
            advantage_l = m_acc_l - m_acc_c
            advantage_r = m_acc_r - m_acc_c + a_bias
            disadvantage_l = vehicle.p * (b_acc_c_1 + b_acc_l_1 - b_acc_c_2 - b_acc_l_2) + thresh_l
            disadvantage_r = vehicle.p * (b_acc_c_1 + b_acc_r_1 - b_acc_c_2 - b_acc_r_2) + thresh_r

            # if the vehicle can move right and the citeria are satisfied, moves right
            # otherwise, if the criteria to move left are satisfied, moves left
            acc = m_acc_c
            bias = False
            if bias:
                if vehicle.lane > 0 and b_acc_r_2 > b_safe and advantage_r > disadvantage_r:
                    vehicle.lane -= 1
                    acc = m_acc_r
                elif vehicle.lane < n_lanes - 1 and b_acc_l_2 > b_safe and advantage_l > disadvantage_l:
                    vehicle.lane += 1
                    acc = m_acc_l
            else:
                r_cand = l_cand = False
                if vehicle.lane > 0 and b_acc_r_2 > b_safe and advantage_r > disadvantage_r - thresh_r + thresh_l:
                    r_cand = True
                if vehicle.lane < n_lanes - 1 and b_acc_l_2 > b_safe and advantage_l > disadvantage_l:
                    l_cand = True
                if (r_cand and not l_cand) or (r_cand and advantage_r - disadvantage_r + thresh_r - thresh_l > advantage_l - disadvantage_l):
                    vehicle.lane -= 1
                    acc = m_acc_r
                elif l_cand:
                    vehicle.lane += 1
                    acc = m_acc_l
 

            vehicle.x += h * vehicle.v
            vehicle.v += h * acc

            # remove vehicle if past simulation boundary
            if(vehicle.x > x_end):
                vehicles.remove(vehicle)
            else:
                i += 1


def main():
    sim = TrafficSimulation(2)
    sim.gen_models[1].delta = 15.
    n_frames = 3300 # at h = 0.2 yields 8:20 sim time

    # spatial data collection interval
    x_start = sim.x_start + 1600.
    x_end = sim.x_end - 400. # we collect data along a 3000m stretch of highway
    
    sig_v = np.zeros(n_frames)
    sig_x = np.zeros(n_frames)

    # set up figure and animation
    fig = plt.figure()
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    ax = fig.add_subplot(211, autoscale_on=False, xlim=(0, sim.x_end), ylim=(-.5, sim.n_lanes + 0.5))
    pos, = ax.plot([], [], 'bo', ms=6.)
    vel, = ax.plot([], [], 'go', ms=2.)

    def init_anim():
        pos.set_data([], [])
        vel.set_data([], [])
        return [],

    def animate(i):
        sim.step(i)
        vehicles = sim.vehicles
        x = [v.x for v in vehicles]
        lanes = [float(v.lane) for v in vehicles]
        dx = [sim.n_lanes * v.v / 60. + v.lane - 1 for v in vehicles]
        pos.set_data(x,lanes)
        vel.set_data(x, dx)
        return pos, vel,

    ani = animation.FuncAnimation(fig, animate, frames=n_frames, interval=2, blit=False, init_func=init_anim)
    plt.show()

    

if __name__ == "__main__":
    main()

