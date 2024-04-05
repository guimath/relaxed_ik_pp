from typing import List


class RelaxedWrapper:
    """
    A class representing relaxedIK solver.

    Parameters
    ----------
    path_to_setting: path to the setting file (E.G. 'configs/settings.yaml')
    """
    def __init__(self, path_to_setting: str) -> None: ...

    def solve_position(self, pos_goals:List[float], quat_goals:List[float], tolerances:List[float]) -> tuple[List[float], float]:
        """
        Solves IK for provided pose

        Parameters
        ----------
        Assuming the robot has N end-effectors
        pos_goals: (1D array with length as 3*N) list of end-effector positions
        quat_goals: (1D array with length as 4*N) list of end-effector orientations (in quaternion xyzw format)
        tolerances: (1D array with length as 6*N) list of tolerances for each end-effector (x, y, z, rx, ry, rz)

        Returns
        ----------
        tuple (joints position at end of optimization, cost)
        """
    
    def reset_origin(self) -> None : 
        ''' Resets joint positions to origin''' 

    def get_ee_pos(self) -> tuple[List[float], List[float]]:
        ''' 
        Gets the pos of the end effector
        
         Returns
        ----------
        EE coordinates (XYZ) in a list
        EE quaternion (WIJK) in a list
        '''

    def get_objectives_costs(self) -> List[float]:
        ''' Gets cost per objective'''