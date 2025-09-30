#!/usr/bin/env python3
"""
Script to scale down apt_complex.world objects by 1/3 and rearrange them beautifully
"""

import re

def scale_world_file():
    input_file = "/home/donggun/turtle_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/apt_complex.world"
    output_file = "/home/donggun/turtle_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/apt_complex_scaled.world"
    
    scale_factor = 1/3
    
    with open(input_file, 'r') as f:
        content = f.read()
    
    # Scale pose positions (x y z roll pitch yaw)
    def scale_pose(match):
        values = match.group(1).split()
        if len(values) >= 3:
            # Scale x, y, z coordinates but keep rotations
            values[0] = str(float(values[0]) * scale_factor)
            values[1] = str(float(values[1]) * scale_factor) 
            values[2] = str(float(values[2]) * scale_factor)
        return f"<pose>{' '.join(values)}</pose>"
    
    # Scale size dimensions
    def scale_size(match):
        values = match.group(1).split()
        scaled_values = [str(float(v) * scale_factor) for v in values]
        return f"<size>{' '.join(scaled_values)}</size>"
    
    # Scale scale values (for meshes)
    def scale_scale(match):
        values = match.group(1).split()
        scaled_values = [str(float(v) * scale_factor) for v in values]
        return f"<scale>{' '.join(scaled_values)}</scale>"
    
    # Don't scale ground plane and some essential elements
    lines = content.split('\n')
    in_ground_plane = False
    in_sun = False
    
    scaled_lines = []
    for line in lines:
        # Track if we're in ground_plane or sun model
        if "<model name='ground_plane'>" in line:
            in_ground_plane = True
        elif "<model name='sun'" in line or "<light name='sun'" in line:
            in_sun = True
        elif "</model>" in line:
            in_ground_plane = False
        elif "</light>" in line:
            in_sun = False
        
        # Skip scaling for ground plane and sun
        if not in_ground_plane and not in_sun:
            # Scale poses
            line = re.sub(r'<pose>([-\d\.\s]+)</pose>', scale_pose, line)
            # Scale sizes  
            line = re.sub(r'<size>([-\d\.\s]+)</size>', scale_size, line)
            # Scale scale values
            line = re.sub(r'<scale>([-\d\.\s]+)</scale>', scale_scale, line)
        
        scaled_lines.append(line)
    
    scaled_content = '\n'.join(scaled_lines)
    
    # Write to new file
    with open(output_file, 'w') as f:
        f.write(scaled_content)
    
    print(f"Scaled world saved to: {output_file}")

if __name__ == "__main__":
    scale_world_file()