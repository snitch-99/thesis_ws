import numpy as np

def generate_orbit(center_x, center_y, radius, height, angular_resolution, home_x=0.0, home_y=0.0):
    """
    Generates a list of (x, y, z) waypoints for a circular orbit.
    The list is reordered to start with the waypoint closest to (home_x, home_y).
    
    Args:
        center_x (float): X coordinate of the center.
        center_y (float): Y coordinate of the center.
        radius (float): Radius of the orbit.
        height (float): Altitude (z) of the orbit.
        angular_resolution (float): Angular spacing between points in radians.
        home_x (float): X coordinate of the home/start position.
        home_y (float): Y coordinate of the home/start position.
        
    Returns:
        list: List of tuples [(x, y, z), ...]
    """
    waypoints = []
    
    # Generate angles from 0 to 2*pi
    angles = np.arange(0, 2*np.pi, angular_resolution)
    
    for theta in angles:
        x = center_x + radius * np.cos(theta)
        y = center_y + radius * np.sin(theta)
        z = height
        waypoints.append((x, y, z))
    
    if not waypoints:
        return waypoints

    # Find the index of the waypoint closest to home
    min_dist_sq = float('inf')
    closest_index = 0
    
    for i, (wx, wy, _) in enumerate(waypoints):
        dist_sq = (wx - home_x)**2 + (wy - home_y)**2
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_index = i

    sorted_waypoints = waypoints[closest_index:] + waypoints[:closest_index]
        
    return sorted_waypoints
