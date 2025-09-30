#!/usr/bin/env python3
"""
Script to properly scale down apt_complex.world objects by 1/3 (positions and sizes only, not rotations)
"""

import re

def fix_scale_world_file():
    # First restore from install directory
    input_file = "/home/donggun/turtle_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds/apt_complex.world"
    output_file = "/home/donggun/turtle_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/apt_complex.world"
    
    scale_factor = 1/3
    
    with open(input_file, 'r') as f:
        content = f.read()
    
    # Scale pose positions (x y z roll pitch yaw) - only scale x, y, z
    def scale_pose(match):
        values = match.group(1).split()
        if len(values) >= 3:
            # Scale x, y, z coordinates but keep rotations unchanged
            values[0] = str(float(values[0]) * scale_factor)
            values[1] = str(float(values[1]) * scale_factor) 
            values[2] = str(float(values[2]) * scale_factor)
            # Keep roll, pitch, yaw as-is (values[3:])
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
    
    # Don't scale ground plane, sun, and some essential elements
    lines = content.split('\n')
    in_ground_plane = False
    in_sun = False
    in_physics = False
    
    scaled_lines = []
    for line in lines:
        # Track if we're in ground_plane, sun, or physics sections
        if "<model name='ground_plane'>" in line:
            in_ground_plane = True
        elif "<light name='sun'" in line or "<light type='directional'>" in line:
            in_sun = True
        elif "<physics" in line:
            in_physics = True
        elif "</model>" in line:
            in_ground_plane = False
        elif "</light>" in line:
            in_sun = False
        elif "</physics>" in line:
            in_physics = False
        
        # Skip scaling for ground plane, sun, and physics
        if not in_ground_plane and not in_sun and not in_physics:
            # Scale poses (only x, y, z)
            line = re.sub(r'<pose>([-\d\.\s]+)</pose>', scale_pose, line)
            # Scale sizes  
            line = re.sub(r'<size>([-\d\.\s]+)</size>', scale_size, line)
            # Scale scale values
            line = re.sub(r'<scale>([-\d\.\s]+)</scale>', scale_scale, line)
        
        scaled_lines.append(line)
    
    scaled_content = '\n'.join(scaled_lines)
    
    # Write back to source file
    with open(output_file, 'w') as f:
        f.write(scaled_content)
    
    print(f"Fixed scaled world saved to: {output_file}")

if __name__ == "__main__":
    fix_scale_world_file()