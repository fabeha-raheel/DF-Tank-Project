import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, AutoMinorLocator, FuncFormatter
def plot_compass(angle):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

    # Draw the compass circle
    theta = np.radians(np.arange(0, 360, 1))  # One tick for each degree
    ax.plot(theta, np.ones_like(theta), color='black', linewidth=2)

    # Mark angles from 0 to 360 degrees with major ticks at 45-degree intervals
    ax.set_xticks(np.radians(np.arange(0, 360, 45)))

    # Add minor ticks at 1-degree intervals
    ax.xaxis.set_minor_locator(AutoMinorLocator(4))

    # Hide radial labels and lines
    ax.set_yticklabels([])
    ax.set_yticks([])

    # Draw the needle
    ax.arrow(np.radians(angle), 0.5, 0, 0.4, width=0.05, color='red', alpha=0.7)

    # Set 0 degree at the top
    ax.set_theta_offset(np.pi/2)
    ax.set_theta_direction(-1)

    plt.show()

# Example: Plotting the compass with a needle at 45 degrees
plot_compass(45)
