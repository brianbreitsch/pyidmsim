import random
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import simulation as trafficsim

sim = trafficsim.TrafficSimulation(3)
sim.gen_models[1].delta = 15.
sim.gen_models[2].delta = 30.
n_frames = 3300 # at h = 0.2 yields 8:20 sim time

# spatial data collection interval
x_start = sim.x_start + 1600.
x_end = sim.x_end - 400. # we collect data along a 3000m stretch of highway

sig_v = np.zeros(n_frames)
sig_x = np.zeros(n_frames)

hist_bins = [1., 2., 4., 8., 16.]
n_bins = len(hist_bins)
safe_hist = np.zeros((n_bins, n_frames))

for i in range(n_frames):
    sim.step(i)
    vehicles = sim.vehicles
    sig_v[i] = trafficsim.comp_sigma_v(sim.vehicles, x_start, x_end)
    sig_x[i] = trafficsim.comp_sigma_x(sim.vehicles, x_start, x_end)
    safe_hist[:,i] = trafficsim.comp_safety_hist(sim.vehicles, x_start, x_end, hist_bins)


fig, host = plt.subplots()
fig.subplots_adjust(right=0.75)

par1 = host.twinx()

p1, = host.plot(np.arange(n_frames) * sim.h, sig_x, 'g-', label="Sigma X Index")
p2, = par1.plot(np.arange(n_frames) * sim.h, sig_v, 'r-', label="Sigma V Index")

host.set_xlim(200., 700.)
host.set_ylim(0, 500.)
par1.set_ylim(-10., 1.)

host.set_xlabel("Simulation Time (s)")
host.set_ylabel("Distance (m)")
par1.set_ylabel("Velocity (m/s)")

lines = [p1, p2]
host.legend(lines, [l.get_label() for l in lines])

# adjust far par
par1.set_frame_on(True)
par1.patch.set_visible(False)
plt.setp(par1.spines.values(), visible=False)
par1.spines["right"].set_visible(True)




# make hist figure
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')
#ax2.set_xlim(200., 700.)

xs = np.arange(n_frames - 1200) * sim.h + 1200. * sim.h
for i, c in zip(range(5), ['b', 'g', 'y', 'r', 'm']):
    ys = safe_hist[i,1200:]
    z = hist_bins[i]
    ax2.bar(xs, ys, zs=z, zdir='y', color=c, edgecolor='none')


ax2.set_xlabel('Simulation Epoch (s)')
ax2.set_ylabel('Time Heading Slot (s)')
ax2.set_zlabel('Number of Vehicles')

plt.show()

