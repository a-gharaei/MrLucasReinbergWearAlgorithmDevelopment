from Tutorials.ProcessSimualtionWalkthrough.process_simulation_walkthrough import *


new_pose = Pose.identity()
triangle_1 = Triangle(Vector(1, 0, 1), Vector(1, 0, 0), Vector(0, 0, 0))
mesh_1 = Mesh([triangle_1], new_pose)
mesh_2 = Mesh([triangle_1], new_pose)

# Move the tool

tool_moved = tool.move(Pose.identity())
