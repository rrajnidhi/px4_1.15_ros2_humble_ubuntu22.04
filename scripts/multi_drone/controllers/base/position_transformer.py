from typing import Union, Literal
import numpy as np
from multi_drone.controllers.base.base_data import (
    PositionData,
    OrientationData,
    VelocityData,
)
from multi_drone_msg.msg import LocalAndGlobalCoordinatesMsg


class DroneLocalityState():
    """
    Class for managing the drone's state in different coordinate systems (NED and ENU).
    Attributes:
    ----------
    - position_local_ENU: Position in the local ENU system.
    - velocity_local_ENU: Velocity in the local ENU system.
    - position_local_NED: Position in the local NED system.
    - velocity_local_NED: Velocity in the local NED system.
    - position_global_ENU: Position in the global ENU system.
    - velocity_global_ENU: Velocity in the global ENU system.
    - position_global_NED: Position in the global NED system.
    - velocity_global_NED: Velocity in the global NED system.
    - world_position_ENU: World position in ENU coordinates.
    - world_orientation_ENU: World orientation in ENU coordinates.
    - yaw_NED: Yaw angle in the NED system.
    - yaw_ENU: Yaw angle in the ENU system.
    """
   
    def __init__(
        self,
        world_position = PositionData(x=0, y=0, z=0),
        world_orientation = OrientationData(roll=0, pitch=0, yaw=0)
    ):
        """
        Initializes the drone state with the given position and orientation.
        Parameters:
        ----------
        - world_position (PositionData): Initial position in the global ENU system.
        - world_orientation (OrientationData): Initial orientation in the global ENU system.
        """
       
        self.position_local_ENU = PositionData()
        self.velocity_local_ENU = VelocityData()
        # self.orientation_local_ENU = OrientationData()
       
        self.position_local_NED = PositionData()
        self.velocity_local_NED = VelocityData()
        # self.orientation_local_NED = OrientationData()
       
        self.position_global_ENU = PositionData()
        self.velocity_global_ENU = VelocityData()
        # self.orientation_global_ENU = OrientationData()
       
        self.position_global_NED = PositionData()
        self.velocity_global_NED = VelocityData()
        # self.orientation_global_NED = OrientationData()
       
        self.world_position_ENU = world_position
        self.world_orientation_ENU = world_orientation
       
        self.yaw_NED = 0.0
        self.yaw_ENU = 0.0
       
    def get_position(
        self,
        system: Literal[
            "local_NED", "local_ENU", 'global_ENU', 'global_NED'
        ] = 'local_NED'
    ) -> np.ndarray:
       
        if system == 'local_NED':
            return self.position_local_NED.to_array()
        elif system == 'local_ENU':
            return self.position_local_ENU.to_array()
        elif system == 'global_ENU':
            return self.position_global_ENU.to_array()
        elif system == 'global_NED':
            return self.position_global_NED.to_array()
        else:
            raise ValueError(f"Unknown system: {system}")
   
    def get_velocity(
        self,
        system: Literal[
            "local_NED", "local_ENU", 'global_ENU', 'global_NED'
        ] = 'local_NED'
    ) -> np.ndarray:
       
        if system == 'local_NED':
            return self.velocity_local_NED.to_array()
        elif system == 'local_ENU':
            return self.velocity_local_ENU.to_array()
        elif system == 'global_ENU':
            return self.velocity_global_ENU.to_array()
        elif system == 'global_NED':
            return self.velocity_global_NED.to_array()
        else:
            raise ValueError(f"Unknown system: {system}")
   
    def get_orientation(
        self,
        system: Literal[
            "local_NED", "local_ENU", "global_ENU", "global_NED"
        ] = "local_NED",
    ) -> float:
        """
        Returns the yaw angle in the specified coordinate system.
        Parameters:
        ----------
        - system (Literal): Coordinate system ("local_NED", "local_ENU", "global_ENU", "global_NED").
        Returns:
        ----------
        - float: Yaw angle in radians.
        """
        if system == "local_NED":
            return self.yaw_NED
        elif system == "local_ENU":
            return self.yaw_ENU
        elif system == "global_ENU":
            return self.yaw_ENU
        elif system == "global_NED":
            return self.yaw_NED
        else:
            raise ValueError(f"Unknown system: {system}")
           
    def update_position(
            self,
            array_p: np.ndarray,
            system: Literal[
                    "local_NED", "local_ENU", 'global_ENU', 'global_NED'
                ] = 'local_NED'
        ):
        if system == 'local_NED':
            self.position_local_NED.update_from_array(array_p)
           
            self.position_local_ENU.update_from_array(
                self.position_local_NED.to_ENU())
               
            self.position_global_ENU.update_from_array(
                self.position_local_ENU.to_global(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
           
            self.position_global_NED.update_from_array(
                self.position_global_ENU.to_NED())
           
        elif system == 'local_ENU':
            self.position_local_ENU.update_from_array(array_p)
           
            self.position_local_NED.update_from_array(
                self.position_local_ENU.to_NED())
               
            self.position_global_ENU.update_from_array(
                self.position_local_ENU.to_global(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
           
            self.position_global_NED.update_from_array(
                self.position_global_ENU.to_NED())
           
        elif system == 'global_ENU':
            self.position_global_ENU.update_from_array(array_p)
           
            self.position_local_ENU.update_from_array(
                self.position_global_ENU.to_local(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
           
            self.position_local_NED.update_from_array(
                self.position_local_ENU.to_NED())
           
            self.position_global_NED.update_from_array(
                self.position_global_ENU.to_NED())
           
        elif system == 'global_NED':
            self.position_global_NED.update_from_array(array_p)
           
            self.position_global_ENU.update_from_array(
                self.position_global_NED.to_ENU())
               
            self.position_local_ENU.update_from_array(
                self.position_global_ENU.to_local(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
           
            self.position_local_NED.update_from_array(
                self.position_local_ENU.to_NED())
        else:
            raise ValueError(f"Unknown system: {system}")
           
    def update_velocity(
        self,
        array_v: np.ndarray,
        system: Literal[
                "local_NED", "local_ENU", 'global_ENU', 'global_NED'
            ] = 'local_NED'
    ):
        if system == 'local_NED':
            self.velocity_local_NED.update_from_array(array_v)
           
            self.velocity_local_ENU.update_from_array(
                self.velocity_local_NED.to_ENU())
           
            self.velocity_global_ENU.update_from_array(
                self.velocity_local_ENU.to_global(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
           
            self.velocity_global_NED.update_from_array(
                self.velocity_global_ENU.to_NED())
        elif system == 'local_ENU':
            self.velocity_local_ENU.update_from_array(array_v)
           
            self.velocity_local_NED.update_from_array(
                self.velocity_local_ENU.to_NED())
           
            self.velocity_global_ENU.update_from_array(
                self.velocity_local_ENU.to_global(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
           
            self.velocity_global_NED.update_from_array(
                self.velocity_global_ENU.to_NED())
        elif system == 'global_ENU':
            self.velocity_global_ENU.update_from_array(array_v)
           
            self.velocity_local_ENU.update_from_array(
                self.velocity_global_ENU.to_local(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
           
            self.velocity_local_NED.update_from_array(
                self.velocity_local_ENU.to_NED())
           
            self.velocity_global_NED.update_from_array(
                self.velocity_global_ENU.to_NED())
        elif system == 'global_NED':
            self.velocity_global_NED.update_from_array(array_v)
           
            self.velocity_global_ENU.update_from_array(
                self.velocity_global_NED.to_ENU())
           
            self.velocity_local_ENU.update_from_array(
                self.velocity_global_ENU.to_local(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
           
            self.velocity_local_NED.update_from_array(
                self.velocity_local_ENU.to_NED())
        else:
            raise ValueError(f"Unknown system: {system}")
       
    def update_orientation(
        self,
        array_q: np.ndarray,
        system: Literal[
                "local_NED", "local_ENU", 'global_ENU', 'global_NED'
            ] = 'local_NED'
    ):
       
        # Not yet sure if needed and calculation needs verification
       
        # self.orientation_local_NED.update_from_quaternion_array(array_q)
       
        # self.orientation_local_ENU.update_from_quaternion_array(
        # self.orientation_local_NED.quaternion.to_ENU())
       
        # self.orientation_global_ENU.update_from_quaternion_array(
        # self.orientation_local_ENU.quaternion.to_global(
        # self.world_orientation_ENU.quaternion.to_array()))
       
        # self.orientation_global_NED.update_from_quaternion_array(
        # self.orientation_global_ENU.quaternion.to_NED())
       
        self.yaw_ENU = float(np.arctan2(2.0 * (array_q[3] * array_q[2] + array_q[0] * array_q[1]),
                                  1.0 - 2.0 * (array_q[1] * array_q[1] + array_q[2] * array_q[2])))
        self.yaw_NED = -self.yaw_ENU
       
    def to_msg(self) -> LocalAndGlobalCoordinatesMsg:
        msg = LocalAndGlobalCoordinatesMsg()
        msg.local_enu.position = self.position_local_ENU.to_vector3()
        msg.local_enu.velocity = self.velocity_local_ENU.to_vector3()
        msg.local_enu.yaw = self.yaw_ENU
       
        msg.local_ned.position = self.position_local_NED.to_vector3()
        msg.local_ned.velocity = self.velocity_local_NED.to_vector3()
        msg.local_ned.yaw = self.yaw_NED
       
        msg.global_enu.position = self.position_global_ENU.to_vector3()
        msg.global_enu.velocity = self.velocity_global_ENU.to_vector3()
        msg.global_enu.yaw = self.yaw_ENU
       
        msg.global_ned.position = self.position_global_NED.to_vector3()
        msg.global_ned.velocity = self.velocity_global_NED.to_vector3()
        msg.global_ned.yaw = self.yaw_NED
       
        return msg
