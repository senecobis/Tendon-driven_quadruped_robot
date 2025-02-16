import numpy as np

def arch(samples=100):
    # Parameters for the arch
    radius = 0.1         # Radius of the arch
    height = 0         # Maximum height of the arch
    theta_start = 0    # Starting angle in radians
    theta_end = np.pi  # Ending angle in radians (semi-circle)

    # Generate the theta values for the arch
    theta = np.linspace(theta_start, theta_end, samples)

    # Convert polar coordinates to Cartesian coordinates on the x-y plane
    x = radius * np.cos(theta)[::-1]
    z = radius * np.sin(theta)[::-1]

    # Define z values for the arch
    y = height * np.sin(theta)[::-1]
    return x,y,z