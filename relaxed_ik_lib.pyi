from typing import List


class RelaxedWrapper:
    """
    A class representing relaxedIK solver.

    Parameters
    ----------
    path_to_setting: path to the setting file (E.G. 'configs/settings.toml') Can be absolute or relative to the crate folder.
    """
    def __init__(self, path_to_setting: str) -> None: ...

    def grip(self, pos_goals:List[float]) -> List[List[float]]:
        """
        Creates steps to get to given object position and grasp.

        Parameters
        ----------
        pos_goals: array of length 3, list of end-effector position

        Returns
        ----------
        list of steps (joints position) to reach grasp target
        """
    
    def ik(self, pos_goals:List[float]) -> List[float]:
        """
        Compute IK at given goal

        Parameters
        ----------
        pos_goals: array of length 3, list of end-effector position

        Returns
        ----------
        joint configuration to position EE at given goal
        """

    def reset(self, x=List[float]) -> None :
        ''' Sets joints to given values'''
    
    def reset_origin(self) -> None : 
        ''' Resets joint positions to origin''' 


