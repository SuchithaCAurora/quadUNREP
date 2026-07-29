#!/usr/bin/env python3
"""
cbf_figure8_sim.py

Standalone numerical sim + 3D visualization of the acceleration-level HOCBF
safety filter around a Figure8 mission, mirroring the exact equations used in:
  - uav_trajectory_generator/src/trajectories/Figure8.cpp   (nominal traj)
  - uav_trajectory_generator/src/cbf_safety_filter_node.cpp (CBF filter)

No ROS / PX4 dependency. Purpose: validate the CBF math and hand-tune
alpha1 / alpha2 / obstacle_radius / influence_margin here, cheaply, before
running the real C++ node against PX4 SITL and then hardware.

Workflow: this script -> PX4 SITL (cbf_safety_filter_node.cpp) -> hardware.

Usage:
    python cbf_figure8_sim.py
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)
from matplotlib.animation import FuncAnimation


# ============================================================================
# Nominal Figure8 trajectory generator -- mirrors Figure8.cpp exactly
# ============================================================================

def generate_figure8(alt, r, cx, cy, v_goals, t_traj, accel, dt):
    """Returns p, v, a as (N,3) arrays and t as (N,) -- same ramp/hold/decel
    schedule as Figure8::generateTraj in Figure8.cpp."""

    def goal(v, theta):
        s, c = np.sin(theta), np.cos(theta)
        sc = s * c
        omega = v / r
        p = np.array([cx + r * s, cy + r * sc, alt])
        vel = np.array([r * omega * c, r * omega * (c * c - s * s), 0.0])
        acc = np.array([-r * omega * omega * s, -4.0 * r * omega * omega * sc, 0.0])
        return p, vel, acc

    p_list, v_list, a_list = [], [], []
    theta, v = 0.0, 0.0

    p0, v0, a0 = goal(v, theta)
    p_list.append(p0); v_list.append(v0); a_list.append(a0)

    for v_goal in v_goals:
        while v < v_goal:
            v = min(v + accel * dt, v_goal)
            theta += (v / r) * dt
            p, vel, acc = goal(v, theta)
            p_list.append(p); v_list.append(vel); a_list.append(acc)

        t_hold = 0.0
        while t_hold < t_traj:
            theta += (v / r) * dt
            p, vel, acc = goal(v, theta)
            p_list.append(p); v_list.append(vel); a_list.append(acc)
            t_hold += dt

    while v > 0:
        v = max(v - accel * dt, 0.0)
        theta += (v / r) * dt
        p, vel, acc = goal(v, theta)
        p_list.append(p); v_list.append(vel); a_list.append(acc)

    p_arr = np.array(p_list)
    v_arr = np.array(v_list)
    a_arr = np.array(a_list)
    t_arr = np.arange(len(p_arr)) * dt
    return p_arr, v_arr, a_arr, t_arr


# ============================================================================
# HOCBF acceleration-level safety filter -- mirrors cbf_safety_filter_node.cpp
# ============================================================================

def cbf_accel_projection(p, v, a_nom, obs_center, obs_radius, alpha1, alpha2):
    """Closed-form projection of a_nom onto the HOCBF half-space for one
    sphere. h has relative degree 2 in u (u enters only through v_dot), so
    this enforces psi1_dot + alpha2*psi1 >= 0 rather than a first-order CBF."""
    dp = p - obs_center
    h = dp @ dp - obs_radius ** 2
    psi1 = 2.0 * dp @ v + alpha1 * h

    c = 2.0 * dp
    d = 2.0 * (v @ v) + 2.0 * alpha1 * (dp @ v) + alpha2 * psi1

    cc = c @ c
    margin = c @ a_nom + d
    if margin >= 0.0 or cc < 1e-9:
        return a_nom
    return a_nom - c * (margin / cc)


def run_cbf_filter(p_nom, v_nom, a_nom, dt, obstacles, alpha1, alpha2, margin):
    """obstacles: list of {'center': np.array([x,y,z]), 'radius': r}.
    Filter state only advances inside the influence radius of the nearest
    obstacle; outside it, filtered state is pinned to nominal every step so
    the vehicle re-converges onto the mission once clear (same logic as the
    ROS node)."""
    N = len(p_nom)
    p_out, v_out, a_out = p_nom.copy(), v_nom.copy(), a_nom.copy()
    h_hist = np.full(N, np.nan)

    p_filt = p_nom[0].copy()
    v_filt = v_nom[0].copy()
    active = False

    for k in range(N):
        if not obstacles:
            continue

        dists = [np.linalg.norm(p_nom[k] - o['center']) - o['radius'] for o in obstacles]
        obs = obstacles[int(np.argmin(dists))]
        infl = obs['radius'] + margin
        dist_to_center = np.linalg.norm(p_nom[k] - obs['center'])

        if not active:
            if dist_to_center > infl:
                p_filt, v_filt = p_nom[k].copy(), v_nom[k].copy()
            else:
                active = True

        if active:
            a_safe = cbf_accel_projection(p_filt, v_filt, a_nom[k],
                                           obs['center'], obs['radius'], alpha1, alpha2)
            v_filt = v_filt + a_safe * dt
            p_filt = p_filt + v_filt * dt
            p_out[k], v_out[k], a_out[k] = p_filt, v_filt, a_safe
            h_hist[k] = np.linalg.norm(p_filt - obs['center']) ** 2 - obs['radius'] ** 2

            if np.linalg.norm(p_filt - obs['center']) > infl:
                active = False

    return p_out, v_out, a_out, h_hist


# ============================================================================
# Sphere plotting helper
# ============================================================================

def plot_sphere(ax, center, radius, color='r'):
    u, v = np.mgrid[0:2 * np.pi:24j, 0:np.pi:12j]
    x = center[0] + radius * np.cos(u) * np.sin(v)
    y = center[1] + radius * np.sin(u) * np.sin(v)
    z = center[2] + radius * np.cos(v)
    ax.plot_surface(x, y, z, color=color, alpha=0.3, linewidth=0)


# ============================================================================
# Main
# ============================================================================

def main():
    # --- Figure8 params (mirrors config/default.yaml Figure8 block) ---
    alt, r, cx, cy = 1.8, 3.4, 0.0, 0.0
    v_goals = [2.0, 3.0, 4.0]
    t_traj = 30.0
    accel = 0.4
    dt = 0.01

    # --- CBF params (mirrors cbf_safety_filter_node.cpp defaults) ---
    alpha1, alpha2 = 2.0, 2.0
    influence_margin = 1.0

    # --- Obstacle(s): placed to intersect one lobe of the figure8 ---
    obstacles = [
        {'center': np.array([2.4, 1.2, alt]), 'radius': 0.6},
    ]

    p_nom, v_nom, a_nom, t = generate_figure8(alt, r, cx, cy, v_goals, t_traj, accel, dt)
    p_safe, v_safe, a_safe, h_hist = run_cbf_filter(
        p_nom, v_nom, a_nom, dt, obstacles, alpha1, alpha2, influence_margin)

    # ---------------- static overview: 3D path + barrier value ----------------
    fig = plt.figure(figsize=(12, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection='3d')
    ax3d.plot(p_nom[:, 0], p_nom[:, 1], p_nom[:, 2], '--', color='gray', label='nominal Figure8')
    ax3d.plot(p_safe[:, 0], p_safe[:, 1], p_safe[:, 2], '-', color='tab:blue', label='CBF-filtered')
    for obs in obstacles:
        plot_sphere(ax3d, obs['center'], obs['radius'])
    ax3d.set_xlabel('x [m]'); ax3d.set_ylabel('y [m]'); ax3d.set_zlabel('z [m]')
    ax3d.set_title('Nominal vs. safety-filtered trajectory')
    ax3d.legend()

    axh = fig.add_subplot(1, 2, 2)
    axh.plot(t, h_hist)
    axh.axhline(0.0, color='k', linewidth=0.8)
    axh.set_xlabel('t [s]'); axh.set_ylabel('h(p) = ||p-o||^2 - r^2')
    axh.set_title('Barrier value while filter active (must stay >= 0)')

    fig.tight_layout()

    # ---------------- animated flight ----------------
    fig2 = plt.figure(figsize=(7, 7))
    ax_anim = fig2.add_subplot(111, projection='3d')
    ax_anim.plot(p_nom[:, 0], p_nom[:, 1], p_nom[:, 2], '--', color='gray', linewidth=0.8)
    ax_anim.plot(p_safe[:, 0], p_safe[:, 1], p_safe[:, 2], '-', color='tab:blue', linewidth=0.8)
    for obs in obstacles:
        plot_sphere(ax_anim, obs['center'], obs['radius'])
    nom_pt, = ax_anim.plot([], [], [], 'o', color='gray', markersize=6, label='nominal')
    safe_pt, = ax_anim.plot([], [], [], 'o', color='tab:blue', markersize=6, label='filtered')
    ax_anim.set_xlabel('x [m]'); ax_anim.set_ylabel('y [m]'); ax_anim.set_zlabel('z [m]')
    ax_anim.legend()
    ax_anim.set_title('Figure8 flight: nominal vs. CBF-avoided')

    step = 4  # subsample 100 Hz data for a watchable animation
    frames = range(0, len(t), step)

    def update(k):
        nom_pt.set_data([p_nom[k, 0]], [p_nom[k, 1]])
        nom_pt.set_3d_properties([p_nom[k, 2]])
        safe_pt.set_data([p_safe[k, 0]], [p_safe[k, 1]])
        safe_pt.set_3d_properties([p_safe[k, 2]])
        return nom_pt, safe_pt

    anim = FuncAnimation(fig2, update, frames=frames, interval=dt * step * 1000, blit=False)

    plt.show()
    return anim  # keep a reference so it isn't garbage-collected before show() runs


if __name__ == '__main__':
    _anim = main()
