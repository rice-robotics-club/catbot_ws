import numpy as np
import math

# ik / fk for 4-bar linkage
def calculate_theta3(l0, l1, l2, l3, theta1, open_mode=True):
    """
    Calculates the output angle (theta3) of a four-bar linkage.
    
    Parameters:
    l0, l1, l2, l3: Lengths of the links (l0 is ground)
    theta1_deg: Input angle in degrees
    open_mode: Boolean to choose between the two assembly configurations
    """    
    # Freudenstein Constants
    K1 = l0 / l1
    K2 = l0 / l3
    K3 = (l1**2 - l2**2 + l3**2 + l0**2) / (2 * l1 * l3)
    
    # quadratic coefficients for the half-angle tangent substitution
    # (At^2 + Bt + C = 0)
    A = math.cos(theta1) - K1 - K2 * math.cos(theta1) + K3
    B = -2 * math.sin(theta1)
    C = K1 - (K2 + 1) * math.cos(theta1) + K3
    
    # discriminant check
    discriminant = B**2 - 4*A*C
    if discriminant < 0:
        print("linkage configuration mathematically impossible at this angle!")
        return None  # linkage can't reach this position
    
    # solve for t = tan(theta3 / 2)
    if A == 0:
        print("A==0! Setting to = theta1")
        return math.degrees(theta1)  # linkage can't reach this position

    if open_mode:
        t = (-B + math.sqrt(discriminant)) / (2 * A)
    else:
        t = (-B - math.sqrt(discriminant)) / (2 * A)
        
    theta3 = 2 * math.atan(t)
    return math.degrees(theta3)

def solve_for_th_L12(z, y, L_0):
    L_mag = math.sqrt(z**2 + y**2)
    L_12 = math.sqrt(L_mag**2 - L_0**2)
    
    t_h = math.atan2(z, y) + math.atan2(L_12, L_0)
    t_h = wrap_angle(t_h)
    t_h = math.degrees(t_h) 
    return t_h, L_12

def wrap_angle(rad_angle):
    return (rad_angle + math.pi) % (2 * math.pi) - math.pi

def find_tb_tc(x_EE, z_EE, L_1, L_2):
    if (math.sqrt(x_EE**2 + z_EE**2) > (L_1 + L_2)):
        print("Leg position unreachable!", x_EE, z_EE)
        return None, None
    t_c = math.acos((x_EE**2 + z_EE**2 - L_1**2 - L_2**2) / (2*L_1*L_2))
    t_b = math.atan2(z_EE, x_EE)
    t_b = t_b - math.atan2(L_2*math.sin(t_c), L_1 + L_2 * math.cos(t_c))
    # Wrap to (-pi, pi]
    wrapped_t_b = wrap_angle(t_b)
    return wrapped_t_b, t_c 

def convert_tb_tc_to_ta_tl(t_b, t_c, l0, l1, l2, l3): # NOTE: l0-->l3 = 4-bar linkage links, with l0 = ground
    t_3 = t_b + math.pi
    t_a = calculate_theta3(l0, l3, l2, l1, t_3, open_mode=False) # degrees NOTE: l3 and l1 are flipped because doing IK instead of FK
    if (t_a is None):
        print("Linkage not solvable!")
        return None, None

    t_a = t_a + 180
    t_l = math.degrees(t_3 + t_c) + 180
    return t_a, t_l

def calculate_leg_ik(target_pos, L_0=3.5, L_1=20.7, L_2=15, l0=8.5, l1=6.2, l2=8.7, l3=16.63):
    """ 
    Given desired position of leg EE and lengths of leg, output angles of leg
    note: this does NOT consider configuration of leg 
    assumptions: 
    1. assuming leg is two linkages in series with output angles = angle at each linkage
    2. L1 != L2
    3. hip and upper_leg axes intersect
    input: 
    1. Desired position of leg EE (w.r.t point of intersection between hip and upper_leg axes). 
       If leg is below the chassis (almost always), Z is negative.
    2. L_0, L_1, L_2 = lengths between the following joints: hip->upper_leg, upper_leg->lower_leg, lower_leg->EE
    output: list of solutions (usually 2) [[hip_angle, upper_angle, lower_angle], [...]]
    3. l0, l1, l2, l3 = linkage lengths, with l0 = ground, and l1 
    """
    x, y, z = target_pos[0], target_pos[1], target_pos[2]

    t_h, L_12 = solve_for_th_L12(z, y, L_0)
    if t_h > 90 or t_h < -90:
        print("Hip angle invalid! - IK not solved", t_h)
        return None, None, None
    
    if z < 0:
        # negate L_12 because converting a scalar val to signed val
        L_12=-L_12
    t_b, t_c = find_tb_tc(x_EE=x, z_EE=L_12, L_1=L_1, L_2=L_2)
    if t_b is None or t_c is None:
        print("Leg angle invalid! - IK not solved")
        return None, None, None
    
    t_a, t_l = convert_tb_tc_to_ta_tl(t_b, t_c, l0, l1, l2, l3)
    
    if t_a is None or t_l is None:
        print("Linkage invalid! - IK not solved")
        return None, None, None

    if t_l > 720:
        t_l = t_l - 360
    
    return t_h, t_a, t_l


# TESTS:

# Z
# |         (X pointing out)
# 0--- Y

#  L0
# ---+
#    |         <-- = L0
#    | 
#    | L12

#  L0
# +---
# |            <-- = -L0
# | 
# | L12  

# print(calculate_leg_ik(target_pos=[-27,-3.5,-20], L_0=-3.5, L_1=20.7, L_2=15, l0=8.5, l1=6.2, l2=8.7, l3=16.63))
# print(calculate_leg_ik(target_pos=[-5,3.5,-10], L_0=3.5, L_1=20.7, L_2=15, l0=8.5, l1=6.2, l2=8.7, l3=16.63))


# print(solve_for_th_L12(z=-4, y=3, L_0=3))
# print(solve_for_th_L12(z=3, y=4, L_0=3))
# print(solve_for_th_L12(z=0, y=5, L_0=3))
# print(solve_for_th_L12(z=-4, y=-3, L_0=-3)) # for legs on the right, since L0 points in opposite dir of Y, L_0 is negative

# print(find_tb_tc(x_EE=0, z_EE=-7, L_1=4, L_2=3))
# print(find_tb_tc(x_EE=7, z_EE=0, L_1=4, L_2=3))
# print(find_tb_tc(x_EE=-7, z_EE=0, L_1=4, L_2=3))


# print(calculate_theta3(l0=5.1, l1=6.2, l2=8.7, l3=16.63, theta1=math.radians(-90), open_mode=True))

