
import time
import numpy as np
from pathlib import Path

from xarm.wrapper import XArmAPI
import logging
import relaxed_ik_lib

ABS_PATH = Path(__file__).parent
CONF_PATH = ABS_PATH / 'configs'/ 'xarm6.toml' 
OPEN_GRIPPER = 2000
CLOSE_GRIPPER = 100

def round_str(lst):
    return [f'{e:+.3f}' for e in lst]

def main():
    FORMAT = '%(levelname)5s %(name)s %(filename)s:%(lineno)d %(message)s'
    logging.basicConfig(format=FORMAT)
    logging.getLogger().setLevel(logging.WARNING)
    logging.getLogger('relaxed_ik_lib').setLevel(logging.WARNING)
    rik  = relaxed_ik_lib.RelaxedWrapper(str(CONF_PATH.absolute()))
    arm = XArmAPI("192.168.1.208")
    arm.clean_error()
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    speed = 50
    arm.clean_gripper_error()
    arm.set_gripper_enable(True)
    arm.set_gripper_position(OPEN_GRIPPER)
    arm.set_servo_angle(angle=[0.1, -1.2, -0.15, 0.1, -0.1, 0.0], speed=speed, wait=True, is_radian=True)
    # print(arm.get_servo_angle())
    target = [0.5,0.2,0.3]
    plans = rik.grip(target)
    for i, plan in enumerate(plans):
        if i == 2 : continue
        plan[-1] = plan[-1]-1.57
        print(f'Step {i} - {round_str(plan)} / {round_str(np.degrees(plan).tolist())}')#  - {np.degrees(plan)}
        arm.set_servo_angle(angle=plan, speed=speed, wait=True, is_radian=True)

    y = input("close gripper? (y/n) ")
    if y == 'y' :
        arm.set_gripper_position(CLOSE_GRIPPER)
        
    time.sleep(0.1)
    target[2] = target[2] + 0.3
    up = rik.ik(target)
    up[-1] = up[-1] - 1.57
    arm.set_servo_angle(angle=up, wait=True, is_radian=True)
    
    HORIZONTAL_TRANSLATION = False
    if HORIZONTAL_TRANSLATION : 
        displacement = 0.6
        NUM_IK = 200
        iks = np.zeros((NUM_IK, 6))
        for i in range(NUM_IK):
            target[1] = target[1]-(displacement/NUM_IK)
            iks[i] = rik.ik(target)
        iks[:, 5] =  iks[:, 5] - 1.57
        
        for ik in iks :
            arm.set_servo_angle(angle=ik.tolist(), speed=200, is_radian=True)
    
    time.sleep(1)
    arm.set_servo_angle(angle=[0.1, -1.2, -0.15, 0.1, -0.1, 0.0], speed=speed, wait=True, is_radian=True)

if __name__ == '__main__' :
    main()