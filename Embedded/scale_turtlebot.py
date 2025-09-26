#!/usr/bin/env python3
"""
Script to scale down TurtleBot3 URDF by 1/3
"""

import re

def scale_turtlebot_urdf():
    input_file = "/home/donggun/turtle_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf"
    
    scale_factor = 1/3
    
    with open(input_file, 'r') as f:
        content = f.read()
    
    # Scale origin xyz positions
    def scale_origin(match):
        values = match.group(1).split()
        if len(values) >= 3:
            values[0] = str(float(values[0]) * scale_factor)
            values[1] = str(float(values[1]) * scale_factor) 
            values[2] = str(float(values[2]) * scale_factor)
            # Keep rpy as is
        return f'<origin xyz="{" ".join(values[:3])}"' + (f' rpy="{" ".join(values[3:])}"' if len(values) > 3 else '') + '/>'
    
    # Scale box sizes
    def scale_box_size(match):
        values = match.group(1).split()
        scaled_values = [str(float(v) * scale_factor) for v in values]
        return f'<box size="{" ".join(scaled_values)}"/>'
    
    # Scale cylinder dimensions
    def scale_cylinder(match):
        content = match.group(0)
        # Scale length
        content = re.sub(r'length="([\d\.-]+)"', lambda m: f'length="{float(m.group(1)) * scale_factor}"', content)
        # Scale radius
        content = re.sub(r'radius="([\d\.-]+)"', lambda m: f'radius="{float(m.group(1)) * scale_factor}"', content)
        return content
    
    # Scale mass values (mass scales with volume, so scale^3)
    def scale_mass(match):
        mass = float(match.group(1))
        scaled_mass = mass * (scale_factor ** 3)
        return f'<mass value="{scaled_mass:.6e}"/>'
    
    # Scale inertia values (inertia scales with mass * length^2, so scale^5)
    def scale_inertia(match):
        inertia = float(match.group(2))
        scaled_inertia = inertia * (scale_factor ** 5)
        return f'{match.group(1)}="{scaled_inertia:.6e}"'
    
    # Apply scaling
    lines = content.split('\n')
    scaled_lines = []
    
    for line in lines:
        # Scale origin xyz (but not rpy)
        line = re.sub(r'<origin xyz="([-\d\.\s]+)"([^>]*)/>', scale_origin, line)
        
        # Scale box sizes
        line = re.sub(r'<box size="([-\d\.\s]+)"/>', scale_box_size, line)
        
        # Scale cylinder dimensions
        line = re.sub(r'<cylinder[^>]*>', scale_cylinder, line)
        
        # Scale mass values
        line = re.sub(r'<mass value="([\d\.-e]+)"/>', scale_mass, line)
        
        # Scale inertia values
        line = re.sub(r'(ixx|ixy|ixz|iyy|iyz|izz)="([\d\.-e]+)"', scale_inertia, line)
        
        scaled_lines.append(line)
    
    scaled_content = '\n'.join(scaled_lines)
    
    # Write back to file
    with open(input_file, 'w') as f:
        f.write(scaled_content)
    
    print(f"TurtleBot3 URDF scaled by 1/3")

if __name__ == "__main__":
    scale_turtlebot_urdf()