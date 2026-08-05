import numpy as np
import matplotlib.pyplot as plt

"""
This script is used to sweep the CBF tuning parameter in the guidance loop.
Remember that this is just the velocity augmentation.
This example is for just one axis.
"""

def compute_lambdas(vel_diff, pos_diff, d_max, alpha):
    """
    pos_diff = x_F - x_L
    vel_diff = v_F - v_L
    """
    lambda1 = 2 * max(0.0, vel_diff + alpha * (pos_diff - d_max))
    lambda2 = 2 * max(0.0, -vel_diff + alpha * (-pos_diff - d_max))
    return lambda1, lambda2

def compute_pi(lambda1, lambda2):
    return 0.5 * (lambda2 - lambda1)

def alphaPos_diff():
    max_speed = 1.5
    alpha_array = np.linspace(0.01, 3.0, 50)
    pos_diff_array = np.linspace(-5.0, 5.0, 100)

    # Nominal velocity of follower
    v_F = 0.5
    v_L = 0.0
    vel_diff = v_F - v_L
    d_max = 2.0

    # Sweep alpha and position difference
    PI = np.zeros((len(alpha_array), len(pos_diff_array)))
    for i, alpha in enumerate(alpha_array):
        for j, pos_diff in enumerate(pos_diff_array):
            lambda1, lambda2 = compute_lambdas(vel_diff, pos_diff, d_max, alpha)
            PI[i, j] = compute_pi(lambda1, lambda2)

    # Plot heatmap
    plt.figure(figsize=(10, 6))
    extent = [pos_diff_array[0], pos_diff_array[-1], alpha_array[0], alpha_array[-1]]
    PI_plot = np.clip(PI, -max_speed, max_speed)
    PI_plot_total_u = np.clip(PI+v_F, -max_speed, max_speed)
    plt.imshow(
        PI_plot,
        origin='lower',
        aspect='auto',
        extent=extent,
        cmap='coolwarm'
    )
    plt.colorbar(label=r'$\pi + V_F$')
    plt.xlabel(r'Position difference $x_F - x_L$')
    plt.ylabel(r'$\alpha$')
    plt.title(r'CBF augmented velocity command vs position difference and $\alpha$')

    plt.axvline(d_max, color='k', linestyle='--', linewidth=1, label=r'$d_{max}$')
    plt.axvline(-d_max, color='k', linestyle='--', linewidth=1)

    plt.legend(loc='upper right')

    plt.tight_layout()
    plt.show()


def v_FPos_diff():
    max_speed = 1.5
    v_F_array = np.linspace(-max_speed, max_speed, 50)
    pos_diff_array = np.linspace(-5.0, 5.0, 100)

    v_L = 0.0
    d_max = 0.5
    alpha = 0.20

    # Sweep alpha and position difference
    PI = np.zeros((len(v_F_array), len(pos_diff_array)))
    for i, v_F in enumerate(v_F_array):
        for j, pos_diff in enumerate(pos_diff_array):
            lambda1, lambda2 = compute_lambdas(v_F - v_L, pos_diff, d_max, alpha)
            PI[i, j] = compute_pi(lambda1, lambda2)

    # Plot heatmap
    plt.figure(figsize=(10, 6))
    extent = [pos_diff_array[0], pos_diff_array[-1], v_F_array[0], v_F_array[-1]]
    PI_plot = np.clip(PI, -max_speed, max_speed)
    plt.imshow(
        PI_plot,
        origin='lower',
        aspect='auto',
        extent=extent,
        cmap='coolwarm'
    )
    plt.colorbar(label=r'$\pi$')
    plt.xlabel(r'Position difference $x_F - x_L$')
    plt.ylabel(r'v_{F}')
    plt.title(fr'CBF augmentation vs position difference and nominal command, $\alpha = $ {alpha}')

    plt.axvline(d_max, color='k', linestyle='--', linewidth=1, label=r'$d_{max}$')
    plt.axvline(-d_max, color='k', linestyle='--', linewidth=1)

    plt.legend(loc='upper right')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    alphaPos_diff()
    v_FPos_diff()