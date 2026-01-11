import numpy as np

def generate_trajectory(trajectory_type, center_x, center_y, radius, height, angular_resolution=0.1, step_size=0.5, home_x=0.0, home_y=0.0):
    """
    Generates a list of (x, y, z) waypoints for a specified trajectory shape.
    The list is reordered to start with the waypoint closest to (home_x, home_y).
    
    Args:
        trajectory_type (int): 0 for Circle, 1 for Square.
        center_x (float): X coordinate of the center.
        center_y (float): Y coordinate of the center.
        radius (float): Radius for circle, or half-side length for square.
        height (float): Altitude (z) of the trajectory.
        angular_resolution (float): Angular spacing (radians) for circle generation. Ignored for square.
        step_size (float): Step size (meters) for square edge interpolation. Ignored for circle.
        home_x (float): X coordinate of the home/start position.
        home_y (float): Y coordinate of the home/start position.
        
    Returns:
        list: List of tuples [(x, y, z), ...]
    """
    waypoints = []
    
    if trajectory_type == 0:
        waypoints = _generate_circle_waypoints(center_x, center_y, radius, height, angular_resolution)
    elif trajectory_type == 1:
        waypoints = _generate_square_waypoints(center_x, center_y, radius, height, step_size)
    else:
        # Default to circle if unknown
        waypoints = _generate_circle_waypoints(center_x, center_y, radius, height, angular_resolution)
    
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
    
    # Insert home position at the start
    sorted_waypoints.insert(0, (home_x, home_y, height))
        
    return sorted_waypoints

def _generate_circle_waypoints(center_x, center_y, radius, height, angular_resolution):
    """Generates circular orbit waypoints."""
    waypoints = []
    # Generate angles from 0 to 2*pi
    angles = np.arange(0, 2*np.pi, angular_resolution)
    
    for theta in angles:
        x = center_x + radius * np.cos(theta)
        y = center_y + radius * np.sin(theta)
        z = height
        waypoints.append((x, y, z))
    return waypoints

def _generate_square_waypoints(center_x, center_y, radius, height, step_size):
    """
    Generates square trajectory waypoints with interpolation.
    Radius is treated as half the side length.
    """
    waypoints = []
    # Define corners in order (Loop: TR -> TL -> BL -> BR -> TR)
    corners = [
        (center_x + radius, center_y + radius), # TR
        (center_x - radius, center_y + radius), # TL
        (center_x - radius, center_y - radius), # BL
        (center_x + radius, center_y - radius), # BR
        (center_x + radius, center_y + radius)  # Back to TR to close loop
    ]
    
    for i in range(len(corners) - 1):
        start = corners[i]
        end = corners[i+1]
        
        dist = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        num_steps = int(np.ceil(dist / step_size))
        
        # Linear interpolation
        xs = np.linspace(start[0], end[0], num_steps, endpoint=False) # Endpoint false to avoid duplicate corners
        ys = np.linspace(start[1], end[1], num_steps, endpoint=False)
        
        for x, y in zip(xs, ys):
            waypoints.append((x, y, height))
            
    return waypoints
