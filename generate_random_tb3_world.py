import random

base_world = "/home/anshul/robot_mapping_ws_cpp/worlds/turtlebot3_world_fast.world"
output_world = "/home/anshul/robot_mapping_ws_cpp/worlds/random_tb3.world"

with open(base_world, "r") as f:
    world = f.read()

walls = ""

for i in range(15):

    x = random.uniform(-3.5, 3.5)
    y = random.uniform(-3.5, 3.5)

    if abs(x) < 1.0 and abs(y) < 1.0:
        continue

    # Avoid spawning walls near the robot spawn point (-2.0, -0.5)
    if (x - (-2.0))**2 + (y - (-0.5))**2 < 1.5**2:
        continue

    if random.random() < 0.5:
        sx, sy = 2.0, 0.15
    else:
        sx, sy = 0.15, 2.0

    walls += f'''
    <model name="random_wall_{i}">
      <static>true</static>
      <pose>{x} {y} 0.5 0 0 0</pose>

      <link name="link">

        <collision name="collision">
          <geometry>
            <box>
              <size>{sx} {sy} 1.0</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>{sx} {sy} 1.0</size>
            </box>
          </geometry>
        </visual>

      </link>
    </model>
'''
world = world.replace("</world>", walls + "\n</world>")

with open(output_world, "w") as f:
    f.write(world)

print("Generated:", output_world)
