#!/usr/bin/env python3
"""
cbf_parameter_sweep.py

Parameter sweeps for the CBF safety filter (alpha1/alpha2, mission speed,
obstacle radius) against the Figure8 obstacle-avoidance scenario. Produces
summary tables (stdout) and PNGs (./sweep_plots/) for design-review slides.

Mirrors cbf_figure8_sim.py's generate_figure8()/run_cbf_filter() exactly, so
conclusions here transfer directly to cbf_safety_filter_node.cpp's params
(cbf_alpha1, cbf_alpha2, obstacle_radius).

MAX_VEHICLE_ACCEL below is a real physical constraint, not a tuning knob --
if a sweep point needs more than this, no choice of alpha1/alpha2 makes it
achievable on hardware; PX4's MPC_ACC_HOR will saturate and the vehicle will
penetrate the obstacle by however much the achieved acceleration falls short.
Update it to match your platform once measured (X500 default here is a
conservative estimate from the formation-setpoint-filter design notes).

Run:
    python cbf_parameter_sweep.py
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from cbf_figure8_sim import generate_figure8, run_cbf_filter

# ---------------- scenario defaults (mirrors config/default.yaml) ----------
ALT, R, CX, CY = 1.8, 3.4, 0.0, 0.0
T_TRAJ, ACCEL, DT = 30.0, 0.4, 0.01
OBSTACLE_CENTER = np.array([2.4, 1.2, ALT])

MAX_VEHICLE_ACCEL = 5.0  # [m/s^2] -- update once measured / matched to MPC_ACC_HOR

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sweep_plots')
os.makedirs(OUT_DIR, exist_ok=True)

# Okabe-Ito colorblind-safe subset. Fixed roles, never reused for other series.
COLOR_NOMINAL = '#56B4E9'  # sky blue   -- unfiltered/nominal trajectory
COLOR_SAFE    = '#0072B2'  # deep blue  -- CBF-filtered result
COLOR_THRESH  = '#D55E00'  # vermillion -- infeasibility / penetration threshold
COLOR_GRID    = '#DDDDDD'


def _style_axes(ax):
    ax.grid(True, color=COLOR_GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)


def _evaluate(p_nom, v_nom, a_nom, obstacles, alpha1, alpha2, wn_track=6.0):
    p_safe, v_safe, a_safe, h_hist = run_cbf_filter(
        p_nom, v_nom, a_nom, DT, obstacles, alpha1, alpha2, wn_track)
    peak_a_safe = float(np.max(np.linalg.norm(a_safe, axis=1)))
    peak_a_nom = float(np.max(np.linalg.norm(a_nom, axis=1)))
    min_h = float(np.nanmin(h_hist))
    r = obstacles[0]['radius']
    penetrated = min_h < 0.0
    min_dist = float(np.sqrt(max(min_h + r ** 2, 0.0)))
    return dict(peak_a_safe=peak_a_safe, peak_a_nom=peak_a_nom,
                min_dist=min_dist, penetrated=penetrated)


def _save(fig, name):
    fig.tight_layout()
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=150)
    print(f"saved {path}")


# ============================================================================
def sweep_alpha(v_goal=2.0, obs_radius=0.6, alphas=(0.5, 1.0, 2.0, 4.0, 8.0, 16.0)):
    p_nom, v_nom, a_nom, _ = generate_figure8(ALT, R, CX, CY, [v_goal], T_TRAJ, ACCEL, DT)
    obstacles = [{'center': OBSTACLE_CENTER, 'radius': obs_radius}]
    rows = [_evaluate(p_nom, v_nom, a_nom, obstacles, a, a) for a in alphas]

    print(f"\n=== alpha sweep (v_goal={v_goal} m/s, obstacle r={obs_radius} m) ===")
    print(f"{'alpha1=alpha2':>14} {'peak |a_safe|':>14} {'min surf dist':>14} {'feasible?':>10}")
    for a, row in zip(alphas, rows):
        feas = 'yes' if (row['peak_a_safe'] <= MAX_VEHICLE_ACCEL and not row['penetrated']) else 'NO'
        print(f"{a:>14.2f} {row['peak_a_safe']:>14.2f} {row['min_dist']:>14.3f} {feas:>10}")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    fig.suptitle(f'CBF alpha sweep -- v_goal={v_goal} m/s, obstacle r={obs_radius} m')

    peak = [row['peak_a_safe'] for row in rows]
    ax1.plot(alphas, peak, 'o-', color=COLOR_SAFE, linewidth=2, markersize=6, zorder=3)
    ax1.axhline(MAX_VEHICLE_ACCEL, color=COLOR_THRESH, linestyle='--', linewidth=1.5, zorder=2)
    ax1.text(alphas[-1], MAX_VEHICLE_ACCEL, f'  vehicle max ~{MAX_VEHICLE_ACCEL:.0f} m/s²',
              color=COLOR_THRESH, va='bottom', ha='right', fontsize=9)
    ax1.set_xlabel('alpha1 = alpha2'); ax1.set_ylabel('peak |a_safe| [m/s²]')
    ax1.set_title('Required acceleration'); _style_axes(ax1)

    dist = [row['min_dist'] for row in rows]
    ax2.plot(alphas, dist, 'o-', color=COLOR_SAFE, linewidth=2, markersize=6, zorder=3)
    ax2.axhline(0.0, color=COLOR_THRESH, linestyle='--', linewidth=1.5, zorder=2)
    ax2.text(alphas[-1], 0.0, '  penetration', color=COLOR_THRESH, va='bottom', ha='right', fontsize=9)
    ax2.set_xlabel('alpha1 = alpha2'); ax2.set_ylabel('min distance to surface [m]')
    ax2.set_title('Safety margin'); _style_axes(ax2)

    _save(fig, 'sweep_alpha.png')
    return fig


# ============================================================================
def sweep_speed(alpha=1.0, obs_radius=0.6, v_goals=(1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0)):
    peak_nom, peak_safe, dist = [], [], []
    for v in v_goals:
        p_nom, v_nom, a_nom, _ = generate_figure8(ALT, R, CX, CY, [v], T_TRAJ, ACCEL, DT)
        obstacles = [{'center': OBSTACLE_CENTER, 'radius': obs_radius}]
        row = _evaluate(p_nom, v_nom, a_nom, obstacles, alpha, alpha)
        peak_nom.append(row['peak_a_nom']); peak_safe.append(row['peak_a_safe']); dist.append(row['min_dist'])

    print(f"\n=== speed sweep (alpha1=alpha2={alpha}, obstacle r={obs_radius} m) ===")
    print(f"{'v_goal':>8} {'peak |a_nom|':>14} {'peak |a_safe|':>14} {'min surf dist':>14}")
    for v, pn, ps, md in zip(v_goals, peak_nom, peak_safe, dist):
        print(f"{v:>8.1f} {pn:>14.2f} {ps:>14.2f} {md:>14.3f}")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    fig.suptitle(f'CBF speed sweep -- alpha1=alpha2={alpha}, obstacle r={obs_radius} m')

    ax1.plot(v_goals, peak_nom, 'o-', color=COLOR_NOMINAL, linewidth=2, markersize=6,
              label='nominal (no CBF)', zorder=3)
    ax1.plot(v_goals, peak_safe, 'o-', color=COLOR_SAFE, linewidth=2, markersize=6,
              label='CBF-filtered', zorder=3)
    ax1.axhline(MAX_VEHICLE_ACCEL, color=COLOR_THRESH, linestyle='--', linewidth=1.5, zorder=2)
    ax1.text(v_goals[0], MAX_VEHICLE_ACCEL, f'vehicle max ~{MAX_VEHICLE_ACCEL:.0f} m/s²  ',
              color=COLOR_THRESH, va='bottom', ha='left', fontsize=9)
    ax1.set_xlabel('v_goal [m/s]'); ax1.set_ylabel('peak |a| [m/s²]')
    ax1.set_title('Required acceleration'); ax1.legend(frameon=False); _style_axes(ax1)

    ax2.plot(v_goals, dist, 'o-', color=COLOR_SAFE, linewidth=2, markersize=6, zorder=3)
    ax2.axhline(0.0, color=COLOR_THRESH, linestyle='--', linewidth=1.5, zorder=2)
    ax2.set_xlabel('v_goal [m/s]'); ax2.set_ylabel('min distance to surface [m]')
    ax2.set_title('Safety margin'); _style_axes(ax2)

    _save(fig, 'sweep_speed.png')
    return fig


# ============================================================================
def sweep_obstacle_radius(v_goal=2.0, alpha=1.0, radii=(0.6, 1.0, 1.5, 2.0, 2.5, 3.0)):
    p_nom, v_nom, a_nom, _ = generate_figure8(ALT, R, CX, CY, [v_goal], T_TRAJ, ACCEL, DT)
    peak, dist = [], []
    for rad in radii:
        obstacles = [{'center': OBSTACLE_CENTER, 'radius': rad}]
        row = _evaluate(p_nom, v_nom, a_nom, obstacles, alpha, alpha)
        peak.append(row['peak_a_safe']); dist.append(row['min_dist'])

    print(f"\n=== obstacle radius sweep (v_goal={v_goal} m/s, alpha1=alpha2={alpha}) ===")
    print(f"{'obs_radius':>10} {'peak |a_safe|':>14} {'min surf dist':>14}")
    for rad, pk, md in zip(radii, peak, dist):
        print(f"{rad:>10.1f} {pk:>14.2f} {md:>14.3f}")

    fig, ax1 = plt.subplots(figsize=(6, 4.5))
    fig.suptitle(f'CBF obstacle-radius sweep -- v_goal={v_goal} m/s, alpha1=alpha2={alpha}')
    ax1.plot(radii, peak, 'o-', color=COLOR_SAFE, linewidth=2, markersize=6, zorder=3)
    ax1.axhline(MAX_VEHICLE_ACCEL, color=COLOR_THRESH, linestyle='--', linewidth=1.5, zorder=2)
    ax1.text(radii[-1], MAX_VEHICLE_ACCEL, f'  vehicle max ~{MAX_VEHICLE_ACCEL:.0f} m/s²',
              color=COLOR_THRESH, va='bottom', ha='right', fontsize=9)
    ax1.set_xlabel('obstacle radius [m]'); ax1.set_ylabel('peak |a_safe| [m/s²]')
    ax1.set_title('Radius alone does not fix infeasibility'); _style_axes(ax1)

    _save(fig, 'sweep_obstacle_radius.png')
    return fig


# ============================================================================
def sweep_alpha_vs_speed_heatmap(obs_radius=0.6,
                                  alphas=(0.5, 1.0, 2.0, 4.0, 8.0),
                                  v_goals=(1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0)):
    """2D feasible-operating-envelope map: where does peak |a_safe| stay
    under MAX_VEHICLE_ACCEL, across both gain choice and mission speed."""
    alphas = np.array(alphas, dtype=float)
    v_goals = np.array(v_goals, dtype=float)
    grid = np.full((len(alphas), len(v_goals)), np.nan)

    for i, a in enumerate(alphas):
        for j, v in enumerate(v_goals):
            p_nom, v_nom, a_nom, _ = generate_figure8(ALT, R, CX, CY, [v], T_TRAJ, ACCEL, DT)
            obstacles = [{'center': OBSTACLE_CENTER, 'radius': obs_radius}]
            grid[i, j] = _evaluate(p_nom, v_nom, a_nom, obstacles, a, a)['peak_a_safe']

    fig, ax = plt.subplots(figsize=(7, 5))
    # pcolormesh (not imshow) so the non-uniform alpha spacing renders at its
    # true scale instead of being stretched into evenly-spaced pixels.
    mesh = ax.pcolormesh(v_goals, alphas, grid, shading='nearest', cmap='Blues')
    cbar = fig.colorbar(mesh, ax=ax)
    cbar.set_label('peak |a_safe| [m/s²]')

    if np.nanmin(grid) < MAX_VEHICLE_ACCEL < np.nanmax(grid):
        cs = ax.contour(v_goals, alphas, grid, levels=[MAX_VEHICLE_ACCEL],
                          colors=[COLOR_THRESH], linewidths=2)
        ax.clabel(cs, fmt=f'{MAX_VEHICLE_ACCEL:.0f} m/s² limit', colors=[COLOR_THRESH])

    ax.set_xlabel('v_goal [m/s]'); ax.set_ylabel('alpha1 = alpha2')
    ax.set_title(f'Peak required acceleration -- obstacle r={obs_radius} m')

    _save(fig, 'sweep_alpha_vs_speed_heatmap.png')
    return fig


if __name__ == '__main__':
    sweep_alpha()
    sweep_speed()
    sweep_obstacle_radius()
    sweep_alpha_vs_speed_heatmap()

    print(f"\nAll plots written to {OUT_DIR}/")
    print("Key takeaway so far: mission speed (v_goal) is the dominant lever on "
          "peak required acceleration -- obstacle radius barely moves it, and "
          "pushing alpha1/alpha2 too high (>~10 at 100 Hz) starts violating the "
          "barrier outright rather than helping. See sweep_speed.png first.")

    plt.show()
