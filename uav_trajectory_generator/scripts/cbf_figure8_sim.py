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


def run_cbf_filter(p_nom, v_nom, a_nom, dt, obstacles, alpha1, alpha2, wn_track=6.0):
    """obstacles: list of {'center': np.array([x,y,z]), 'radius': r}.

    The filter continuously runs its own double-integrator state, driven by
    a critically-damped PD tracker of the nominal reference (feedforward
    a_nom plus position/velocity error correction) with the HOCBF projection
    applied on top. Raw a_nom has no dependence on the filtered state, so
    without the tracking term any CBF-induced deviation has zero restoring
    force (error_ddot = 0) and never reconverges -- this is what produced the
    runaway drift and the hard teleport on re-entry to nominal in the first
    version. With the tracker, the filtered state naturally converges onto
    and tracks the nominal trajectory whenever the constraint isn't binding,
    so no discrete active/inactive state machine is needed at all."""
    N = len(p_nom)
    p_out, v_out, a_out = p_nom.copy(), v_nom.copy(), a_nom.copy()
    h_hist = np.full(N, np.nan)

    p_filt = p_nom[0].copy()
    v_filt = v_nom[0].copy()

    for k in range(N):
        a_track = a_nom[k] + wn_track ** 2 * (p_nom[k] - p_filt) + 2.0 * wn_track * (v_nom[k] - v_filt)

        a_safe = a_track
        if obstacles:
            dists = [np.linalg.norm(p_filt - o['center']) - o['radius'] for o in obstacles]
            obs = obstacles[int(np.argmin(dists))]
            a_safe = cbf_accel_projection(p_filt, v_filt, a_track,
                                           obs['center'], obs['radius'], alpha1, alpha2)
            h_hist[k] = np.linalg.norm(p_filt - obs['center']) ** 2 - obs['radius'] ** 2

        v_filt = v_filt + a_safe * dt
        p_filt = p_filt + v_filt * dt
        p_out[k], v_out[k], a_out[k] = p_filt, v_filt, a_safe

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


def set_axes_equal_3d(ax):
    """mplot3d scales each axis independently to fill the plot box, so a
    true sphere renders squashed into a capsule/ellipsoid whenever the data
    ranges differ per axis (exactly the case here: the Figure8 spans ~7m in
    x/y but only ~1-1.5m in z). Force equal 1:1:1 scaling so obstacle spheres
    actually look round. Must be called after all plotting on this axes."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    centers = limits.mean(axis=1)
    radius = 0.5 * np.max(limits[:, 1] - limits[:, 0])
    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])
    ax.set_box_aspect((1, 1, 1))


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
    wn_track = 6.0  # tracking-error natural frequency; must reconverge faster than obstacle encounters recur

    # --- Obstacle(s): placed to intersect one lobe of the figure8 ---
    obstacles = [
        {'center': np.array([2.4, 1.2, alt]), 'radius': 0.6},
    ]

    p_nom, v_nom, a_nom, t = generate_figure8(alt, r, cx, cy, v_goals, t_traj, accel, dt)
    p_safe, v_safe, a_safe, h_hist = run_cbf_filter(
        p_nom, v_nom, a_nom, dt, obstacles, alpha1, alpha2, wn_track)

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
    set_axes_equal_3d(ax3d)

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
    set_axes_equal_3d(ax_anim)

    # mplot3d has no real blitting -- every frame is a full-canvas redraw
    # (~50 ms each measured locally). 'encounter' mode animates just the
    # single closest-approach pass (~10-20s render, best for tuning alpha1/
    # alpha2). 'full_mission' animates the entire flight (~2700 frames even
    # subsampled -> a couple minutes to render) for a complete flythrough --
    # the static overview plot above already shows the whole mission's path
    # instantly if you just want to see the shape, not watch it fly.
    anim_view = 'encounter'  # 'encounter' or 'full_mission'

    pad_s = 2.0
    pad_k = int(pad_s / dt)
    dist_to_nearest_obs = np.min(
        [np.linalg.norm(p_nom - obs['center'], axis=1) - obs['radius'] for obs in obstacles], axis=0)
    encounter_mask = dist_to_nearest_obs < 1.5
    encounter_idx = np.flatnonzero(encounter_mask)

    if anim_view == 'full_mission':
        anim_step = 4  # coarser subsampling -- there's a lot more ground to cover
        window_frames = list(range(0, len(t), anim_step))
        est_s = len(window_frames) * 0.054
        print(f"Animating full mission: {len(window_frames)} frames, "
              f"est. render time ~{est_s:.0f}s ({est_s/60:.1f} min)")
    elif len(encounter_idx) == 0:
        anim_step = 4
        window_frames = list(range(0, len(t), anim_step))  # no encounters -- fall back to full mission
    else:
        anim_step = 2  # 50 Hz playback is visually smooth; halves render time for free
        gaps = np.flatnonzero(np.diff(encounter_idx) > pad_k)
        clusters = np.split(encounter_idx, gaps + 1)

        encounter_choice = int(np.argmin([dist_to_nearest_obs[c].min() for c in clusters]))
        chosen = clusters[encounter_choice]
        lo = max(0, chosen[0] - pad_k)
        hi = min(len(t), chosen[-1] + pad_k)
        window_frames = list(range(lo, hi, anim_step))
        print(f"Animating encounter #{encounter_choice+1}/{len(clusters)}: "
              f"t=[{t[lo]:.2f}, {t[hi-1]:.2f}]s, {len(window_frames)} frames "
              f"(closest approach {dist_to_nearest_obs[chosen].min():.2f} m from surface)")

    def update(k):
        nom_pt.set_data([p_nom[k, 0]], [p_nom[k, 1]])
        nom_pt.set_3d_properties([p_nom[k, 2]])
        safe_pt.set_data([p_safe[k, 0]], [p_safe[k, 1]])
        safe_pt.set_3d_properties([p_safe[k, 2]])
        return nom_pt, safe_pt

    anim = FuncAnimation(fig2, update, frames=window_frames, interval=dt * anim_step * 1000, blit=False)

    plt.show()
    return anim  # keep a reference so it isn't garbage-collected before show() runs


if __name__ == '__main__':
    _anim = main()
