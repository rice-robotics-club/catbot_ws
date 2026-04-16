import numpy as np
import math

# import leg_ik
from enum import Enum

# chassis dims
CHASSIS_WIDTH = 0 # in
CHASSIS_LENGTH = 0 # in

# leg dims
L_0=3.5 
L_1=20.7
L_2=15
l0=8.5
l1=6.2
l2=8.7
l3=16.63 # cm

# init pos of motors at zero position
try_a_init = 110 # deg
t_l_init = 365 # deg

class Gait(Enum):
    STOP = 0
    CRAWL = 1
    TROT = 2
    RL = 3

# simulated motor class
class GhostMotor():

    def __init__(self, id : int):
        self.id = id
        self.pos = 0
        print("Created Motor with id: ", self.id)
 

    def setDesiredPos(self, pos):
        #print("Motor id: ", self.id)
        print("Motor id:", self.id, " Desired Pos set to: ", pos)
        self.pos = pos
        pass

    def getCurrentPos(self):
        return self.pos
    
    def setPos(self, pos): # this HARD SETS the pos - use for zeroing motors.
        print("Motor id: ", self.id)
        print("HARD Set to: ", pos)
        self.pos = pos
        pass


# NOTE: making a gait that's a half-circle overall shape. period will scale with max_ratio
def configure_vel_to_gait_func(step_length, step_width, step_height):
    """
    inputs:
        step_length: [min, max] length of leg step (x)
        step_width: [min, max] width of leg step (y)
        step_height: [min, max] height of leg step (z)
    returns:
        leg_gait(t): a function that, given desired robot vel and arbitrary timestep t, returns a desired pose for the leg. NOTE: t doesn't have to be time in seconds, 
            can slow down and speed up - this is to allow leg to be configured to stay on the ground for longer periods of the leg cycle
    """
    x_stepsize_max = abs(step_length[1] - step_length[0])
    y_stepsize_max = abs(step_width[1] - step_width[0])
    z_stepsize_max = abs(step_height[1] - step_height[0])
    x_midpos = ((step_length[0] + step_length[1]) / 2)
    y_midpos = ((step_width[0] + step_width[1]) / 2)

    def return_gait(vel_input, period_cycle, period_offset, deadzone=0.01):
        """
        inputs:
            vel_input: vector [x, y], for velocity of robot. This means a +x vel will make the leg move "back" at t = 0, dt > 0
            period_cycle: current position in period of gait. [0, 2pi)
            period_offset: offset of gait period

        return:
            pos: given vel_input and current t, input pos of leg
        """


        # if vel input too small, round to 0 and don't move.
        if (abs(vel_input[0]) + abs(vel_input[1]) < deadzone):
            x_pos = 0.0
            y_pos = 0.0
            z_pos = 0.0
        else:
            # period scalar derived from max ratio between input vel and max step size in x, y direction
            period_scalar = max(abs(vel_input[0] / x_stepsize_max), abs(vel_input[1] / y_stepsize_max))

            # current pos in cycle (constant time), [0, 2*pi)
            adj_period_cycle = (period_scalar * period_cycle + period_offset) % (2*math.pi)
            #print("adj_period_cycle before leg ground scaling:" , math.degrees(adj_period_cycle))
            
            # additional scaling to make leg grounded 3/4 of the time
            if adj_period_cycle < math.pi/2:
                adj_period_cycle = adj_period_cycle * 2
            else:
                adj_period_cycle = math.pi + ((adj_period_cycle - math.pi/2) * 2/3)
            #print("adj_period_cycle AFTER leg ground scaling:" , math.degrees(adj_period_cycle))
            # SCALE PERIOD TO ACCOUNT FOR MAX VEL #
            # negation is to make sure robot "steps" backwards when x, y vel is positive (overall robot vel is opposite of leg movement)
            # used to make above math less messy (cos > 0 @ period [0, pi])
            flat_offset = -math.pi/2
            x_pos = math.sin(adj_period_cycle + flat_offset)
            y_pos = math.sin(adj_period_cycle + flat_offset)
            # max bound is for half-circle shape  (0 -> pi/2 & 3pi/2 -> 2pi) < 0
            z_pos = math.cos(adj_period_cycle + flat_offset)
            z_pos = max(z_pos, 0)

            # SCALE by RATIO -> (CURRENT MAX VEL GIVEN period_scalar, INPUT VEL) #
            x_scalar = vel_input[0] / (x_stepsize_max * period_scalar)
            x_pos = x_pos * x_scalar * x_stepsize_max
            
            y_scalar = vel_input[1] / (y_stepsize_max * period_scalar)
            y_pos = y_pos * y_scalar * y_stepsize_max


        # apply offsets for step at t = 0
        x_pos = x_pos + x_midpos
        y_pos = y_pos + y_midpos
        z_pos = z_pos * z_stepsize_max + step_height[0] # height doesn't matter for vel / period scaling

        return x_pos, y_pos, z_pos

    return return_gait


### TEST ###
# step_length = [-6, 6]
# step_width = [-1, 1]
# step_height = [0, 2.5]

# gait_func = configure_vel_to_gait_func(step_length=[-5, -2],step_width=[-2, 2],step_height=[-10, -8])

# for i in [o / 10.0 for o in range(60)]:
#     print(i, gait_func(vel_input=[6, -0.5], period_cycle=i, period_offset=0))

# time_ratio_on_ground=0.75
# time_ratio_on_ground: % of leg gait spent on ground. Used to make sure leg is in air as little as possible while maximizing robot stability
# use this to scale t input, make

