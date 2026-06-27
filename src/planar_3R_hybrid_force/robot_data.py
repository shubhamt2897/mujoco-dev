import numpy as np

class Robot:
    class Body:
        def __init__(self, parent, name, pos, joint_axis, ipos, mass, inertia):
            self.parent = parent
            self.name = name
            self.pos = np.array(pos)
            self.joint_axis = np.array(joint_axis)
            self.ipos = np.array(ipos)
            self.mass = mass
            self.inertia = np.array(inertia)

    class Params:
        """Class to hold robot parameters with heterogeneous data types (scalar/array/matrix)."""
        def __setattr__(self, key, value):
            # Directly assign the value without conversion to preserve data types
            super().__setattr__(key, value)

        def __repr__(self):
            return str(self.__dict__)

    #functions in Robot class
    def __init__(self):
        self.body = {}
        self.params = Robot.Params()  # Use RobotParams for flexible parameter handling

    def add_body(self, body_id, parent, name, pos, joint_axis, ipos, mass, inertia):
        self.body[body_id] = Robot.Body(parent, name, pos, joint_axis, ipos, mass, inertia)


# Initialize the robot
robot = Robot()

# Add body
robot.add_body(
    1, parent='ground', name='link1', pos=[0, 0, 0], joint_axis=[0, 0, 1],
    mass=1, ipos=[0.5, 0, 0], inertia=[1, 1, 0.1, 0, 0, 0]
)

robot.add_body(
    2, parent='link1', name='link2', pos=[1, 0, 0], joint_axis=[0, 0, 1],
    mass=1, ipos=[0.5, 0, 0], inertia=[1, 1, 0.1, 0, 0, 0]
)

robot.add_body(
    3, parent='link2', name='link3', pos=[1, 0, 0], joint_axis=[0, 0, 1],
    mass=0.2, ipos=[0.125, 0, 0], inertia=[0.1, 0.1, 0.02, 0, 0, 0]
)

robot.params.end_eff_pos_local = np.array([0.25,0,0])
robot.params.w = 0.1;
robot.params.h = 0.1;
robot.params.gravity = np.array([0,-9.81,0])

#print(robot.params.end_eff_pos_local)

#example usage
# Add robot-level heterogeneous parameters
# robot.params.array = np.array([0, 0, 0.125])  # Example: np.array
# robot.params.matrix = np.matrix([[1, 2], [3, 4]])  # Example: np.matrix
# robot.params.scalar = 42  # Example: scalar

# Access robot-level parameters
# print("Robot parameters:", robot.params)
# print("Array parameter:", robot.params.array)
# print("Matrix parameter:", robot.params.matrix)
# print("Scalar parameter:", robot.params.scalar)
