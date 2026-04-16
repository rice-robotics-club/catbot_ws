import genesis as gs
import numpy as np
from gait_controller import catbot_crawl

def main():
    # 1. Initialize
    gs.init(backend=gs.gpu)

    # 2. Create Scene
    scene = gs.Scene(show_viewer=True)

    # 3. Add entities
    plane = scene.add_entity(gs.morphs.Plane())
    
    # Import the quadruped (Ensure robot.xml is in your project folder)
    robot = scene.add_entity(
        gs.morphs.MJCF(file='robot.xml', pos=(0.0, 0.0, 1.0))  
    )

    # 4. Build the scene (Required before querying joint indices)
    scene.build()
    
    # 5. Query and match joints
    joint_names = ["FL_hip", "FL_a", "FL_l", "FR_hip", "FR_a", "FR_l", "BL_hip", "BL_a", "BL_l", "BR_hip", "BR_a", "BR_l"]
    dofs_idx = [robot.get_joint(name).dof_idx_local for name in joint_names]
    
    # 6. Run simulation
    while True:
        scene.step()

        angles = catbot_crawl([-2, -2], scene.t / 5.0, turning=True)
        for i in range(4):
            if i ==0 or i ==2:
                angles[i][1] = angles[i][1] - 200
                angles[i][2] = angles[i][2] - 330
            else:
                angles[i][1] = -angles[i][1] + 200
                angles[i][2] = -angles[i][2] + 330


        # angles = [[0,0,0], [0,0,0], [0,0,0], [0,0,0]] #angles[0], angles[0],angles[0]]
        
        rads = np.radians(np.array(angles).flatten())
        robot.control_dofs_position(rads, dofs_idx)
        
        
        # angle names = ["FL_leg", "FR_leg", "BL_leg", "BR_leg"] -> angles = [hip, t_a, t_l]

if __name__ == "__main__":
    main()
