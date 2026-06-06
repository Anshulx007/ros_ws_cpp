#!/usr/bin/env python3

base_world = "/home/anshul/robot_mapping_ws_cpp/worlds/turtlebot3_world_fast.world"
output_world = "/home/anshul/robot_mapping_ws_cpp/worlds/square_room_with_walls.world"

with open(base_world, "r") as f:
    world = f.read()

# Remove the default maze walls model to have a clean slate
world = world.replace("""    <model name="turtlebot3_world">
      <static>1</static>
      <include>
        <uri>model://turtlebot3_world</uri>
      </include>
    </model>""", "")

# Define custom square room and interior walls leaving 1.5m+ gaps
models = [
    # Outer walls (10x10m square room)
    {"name": "outer_north", "x": 0.0, "y": 5.0, "sx": 10.0, "sy": 0.15},
    {"name": "outer_south", "x": 0.0, "y": -5.0, "sx": 10.0, "sy": 0.15},
    {"name": "outer_west", "x": -5.0, "y": 0.0, "sx": 0.15, "sy": 10.0},
    {"name": "outer_east", "x": 5.0, "y": 0.0, "sx": 0.15, "sy": 10.0},
    
    # Inner divider walls (creating 4 rooms connected by 1.5m-2.0m doorways)
    {"name": "inner_vertical_south", "x": 0.0, "y": -3.0, "sx": 0.15, "sy": 4.0},
    {"name": "inner_vertical_north", "x": 0.0, "y": 3.0, "sx": 0.15, "sy": 4.0},
    {"name": "inner_horizontal_east", "x": 3.25, "y": 0.0, "sx": 3.5, "sy": 0.15},
    {"name": "inner_horizontal_west", "x": -3.25, "y": 0.0, "sx": 3.5, "sy": 0.15},

    # Additional obstacles to make mapping more challenging
    {"name": "obs_q1_pillar", "x": 2.5, "y": 2.5, "sx": 1.0, "sy": 1.0},
    {"name": "obs_q2_wall", "x": -2.5, "y": 2.5, "sx": 0.15, "sy": 2.0},
    {"name": "obs_q3_wall", "x": -2.5, "y": -2.5, "sx": 2.0, "sy": 0.15},
    {"name": "obs_q4_pillar", "x": 2.5, "y": -2.5, "sx": 1.0, "sy": 1.0},

    # Even more obstacles
    {"name": "obs_q1_small_pillar", "x": 1.5, "y": 3.5, "sx": 0.6, "sy": 0.6},
    {"name": "obs_q2_small_wall", "x": -3.5, "y": 1.5, "sx": 1.5, "sy": 0.15},
    {"name": "obs_q3_small_box", "x": -1.5, "y": -3.5, "sx": 0.8, "sy": 0.8},
    {"name": "obs_q4_small_wall", "x": 3.5, "y": -1.5, "sx": 0.15, "sy": 1.5},
    {"name": "obs_center_block", "x": 0.0, "y": 0.0, "sx": 0.6, "sy": 0.6},
]

walls_xml = ""
for m in models:
    walls_xml += f"""
    <model name="{m['name']}">
      <static>true</static>
      <pose>{m['x']} {m['y']} 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{m['sx']} {m['sy']} 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{m['sx']} {m['sy']} 1.0</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
"""

# Append the new walls XML before the closing world tag
world = world.replace("</world>", walls_xml + "\n</world>")

with open(output_world, "w") as f:
    f.write(world)

print("Generated:", output_world)
