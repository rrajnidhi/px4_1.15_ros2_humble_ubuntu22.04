import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Vector3, Quaternion

from multi_drone.utils.geometry import (
    rotate_ENU_NED,
    rotated_ENU_NED_quaternion,
    transform_coordinates,
    transform_orientation    
)


class BaseData:
    def _validate_and_convert(self, value):
        if not isinstance(value, (float, int)):
            try:
                value = float(value)
            except ValueError:
                raise TypeError(f"Value {value} cannot be converted to float.")
        return float(value)

    def to_dict(self):
        """
        Forms a dictionary of all object attributes.

        :return: Dictionary with object attributes.
        """
        return {key: value for key, value in self.__dict__.items() if not key.startswith("_")}


class EulerData(BaseData):
    """
    Class for representing orientation using Euler angles (roll, pitch, yaw).

    Attributes:
    - roll (float): Rotation around the X axis.
    - pitch (float): Rotation around the Y axis.
    - yaw (float): Rotation around the Z axis.
    """

    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0):
        # Store data as a numpy array
        self._euler = np.array([self._validate_and_convert(roll),
                                self._validate_and_convert(pitch),
                                self._validate_and_convert(yaw)])

    @property
    def roll(self):
        return self._euler[0]

    @roll.setter
    def roll(self, value):
        self._euler[0] = self._validate_and_convert(value)

    @property
    def pitch(self):
        return self._euler[1]

    @pitch.setter
    def pitch(self, value):
        self._euler[1] = self._validate_and_convert(value)

    @property
    def yaw(self):
        return self._euler[2]

    @yaw.setter
    def yaw(self, value):
        self._euler[2] = self._validate_and_convert(value)

    def to_array(self) -> np.ndarray:
        """
        Returns Euler angles as a numpy array.
        """
        return self._euler.copy()

    def to_vector3(self) -> Vector3:
        """
        Creates a Vector3 object from geometry_msgs.msg.
        """
        return Vector3(x=self.roll, y=self.pitch, z=self.yaw)

    def update(self, roll=None, pitch=None, yaw=None):
        """
        Updates Euler angle values.
        """
        if roll is not None:
            self.roll = roll
        if pitch is not None:
            self.pitch = pitch
        if yaw is not None:
            self.yaw = yaw
    
    def update_from_array(self, euler_array: np.ndarray):
        """
        Updates Euler angles from a numpy array.

        :param euler_array: numpy array of three elements [roll, pitch, yaw].
        """
        if euler_array.shape != (3,):
            raise ValueError(f"Expected array of shape (3,), got {euler_array.shape}")
        
        self._euler = np.array([
            self._validate_and_convert(euler_array[0]),
            self._validate_and_convert(euler_array[1]),
            self._validate_and_convert(euler_array[2])
        ])
        
    def to_quaternion(self) -> np.ndarray:
        """
        Converts Euler angles (roll, pitch, yaw) to a quaternion using scipy.

        :return: numpy array [x, y, z, w] representing the quaternion.
        """
        quaternion = Rotation.from_euler('zyx', self._euler, degrees=False).as_quat()
        return quaternion

    def update_from_quaternion(self, quaternion: np.ndarray):
        """
        Updates Euler angles based on the provided quaternion using scipy.

        :param quaternion: numpy array [x, y, z, w] representing the quaternion.
        """
        if quaternion.shape != (4,):
            raise ValueError(f"Expected array of shape (4,), got {quaternion.shape}")
        euler_angles = Rotation.from_quat(quaternion).as_euler('zyx', degrees=False)
        self.update_from_array(euler_angles)
    
    def to_ENU(self):
        return rotated_ENU_NED_quaternion(self.to_array())
    
    def to_NED(self):
        return rotated_ENU_NED_quaternion(self.to_array())
    
    def to_global(self, reference_orientation):
        return transform_orientation(
            source_orientation=self.to_array(),
            reference_orientation=reference_orientation,
            mode='euler',
            euler_order='zyx',       
            transform_type='local_to_global')
    
    def to_local(self, reference_orientation):
        return transform_orientation(
            source_orientation=self.to_array(),
            reference_orientation=reference_orientation,
            mode='euler',     
            euler_order='zyx',     
            transform_type='global_to_local') 
    
    def __call__(self):
        """
        Returns the orientation as a numpy array when the object is called.
        """
        return self.to_array()

    def __repr__(self):
        return f"EulerData(roll={self.roll}, pitch={self.pitch}, yaw={self.yaw})"


class PositionData(BaseData):
    """
    Class for representing position in space (x, y, z), adapted for use with numpy.
    """

    def __init__(self, x=0.0, y=0.0, z=0.0):
        # Initialize position using a numpy array
        self._position = np.array([self._validate_and_convert(x),
                                   self._validate_and_convert(y),
                                   self._validate_and_convert(z)])

    @property
    def x(self):
        return self._position[0]

    @x.setter
    def x(self, value):
        self._position[0] = self._validate_and_convert(value)

    @property
    def y(self):
        return self._position[1]

    @y.setter
    def y(self, value):
        self._position[1] = self._validate_and_convert(value)

    @property
    def z(self):
        return self._position[2]

    @z.setter
    def z(self, value):
        self._position[2] = self._validate_and_convert(value)

    def to_array(self) -> np.ndarray:
        """
        Returns coordinates as a numpy array.
        """
        return self._position.copy()

    def update(self, x=None, y=None, z=None):
        """
        Updates coordinates if new values are provided.

        :param x: New X coordinate value.
        :param y: New Y coordinate value.
        :param z: New Z coordinate value.
        """
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if z is not None:
            self.z = z

    def update_from_array(self, array: np.ndarray):
        """
        Updates coordinates from a numpy array.

        :param array: numpy array of size 3 containing [x, y, z].
        """
        if not isinstance(array, np.ndarray) or array.shape != (3,):
            raise ValueError("Expected numpy array of size (3,) for [x, y, z].")
        self._position[:] = np.array([
            self._validate_and_convert(array[0]),
            self._validate_and_convert(array[1]),
            self._validate_and_convert(array[2])
        ])

    def to_vector3(self) -> Vector3:
        """
        Creates a Vector3 object from geometry_msgs.msg.

        Returns:
        Vector3: ROS Vector3 object with fields x, y, z.
        """
        return Vector3(x=self.x, y=self.y, z=self.z)
    
    def to_ENU(self):
        return rotate_ENU_NED(self.to_array())
    
    def to_NED(self):
        return rotate_ENU_NED(self.to_array())
    
    def to_global(self, reference_position, reference_orientation, rotation_type='quaternion'):
        return transform_coordinates(
            source_position=self.to_array(),
            reference_position=reference_position,
            reference_orientation=reference_orientation,
            transform_type='local_to_global',
            rotation_type=rotation_type)
    
    def to_local(self, reference_position, reference_orientation, rotation_type='quaternion'):
        return transform_coordinates(
            source_position=self.to_array(),
            reference_position=reference_position,
            reference_orientation=reference_orientation,
            transform_type='global_to_local',
            rotation_type=rotation_type)

    def __call__(self):
        """
        Returns the position as a numpy array when the object is called.
        """
        return self.to_array()
    
    def __repr__(self):
        return f"Position(x={self.x}, y={self.y}, z={self.z})"


class VelocityData(BaseData):
    """
    Class for representing velocity in space (vx, vy, vz), adapted for use with numpy.

    Attributes:
    - vx (float): Velocity along the X axis.
    - vy (float): Velocity along the Y axis.
    - vz (float): Velocity along the Z axis.
    """

    def __init__(self, vx=0.0, vy=0.0, vz=0.0):
        self._velocity = np.array([self._validate_and_convert(vx),
                                   self._validate_and_convert(vy),
                                   self._validate_and_convert(vz)])

    @property
    def vx(self):
        return self._velocity[0]

    @vx.setter
    def vx(self, value):
        self._velocity[0] = self._validate_and_convert(value)

    @property
    def vy(self):
        return self._velocity[1]

    @vy.setter
    def vy(self, value):
        self._velocity[1] = self._validate_and_convert(value)

    @property
    def vz(self):
        return self._velocity[2]

    @vz.setter
    def vz(self, value):
        self._velocity[2] = self._validate_and_convert(value)

    def to_vector3(self) -> Vector3:
        """
        Creates a Vector3 object from geometry_msgs.msg.

        Returns:
        Vector3: ROS Vector3 object with fields vx, vy, vz.
        """
        return Vector3(x=self.vx, y=self.vy, z=self.vz)

    def to_array(self) -> np.ndarray:
        """
        Returns velocity as a numpy array.
        """
        return self._velocity.copy()

    def update(self, vx=None, vy=None, vz=None):
        """
        Updates velocity if new values are provided.

        :param vx: New velocity along X axis.
        :param vy: New velocity along Y axis.
        :param vz: New velocity along Z axis.
        """
        if vx is not None:
            self.vx = vx
        if vy is not None:
            self.vy = vy
        if vz is not None:
            self.vz = vz

    def update_from_array(self, array: np.ndarray):
        """
        Updates velocity from a numpy array.

        :param array: numpy array of size 3 containing [vx, vy, vz].
        """
        if not isinstance(array, np.ndarray) or array.shape != (3,):
            raise ValueError("Expected numpy array of size (3,) for [vx, vy, vz].")
        self._velocity[:] = np.array([
            self._validate_and_convert(array[0]),
            self._validate_and_convert(array[1]),
            self._validate_and_convert(array[2])
        ])
        
    def to_ENU(self):
        return rotate_ENU_NED(self.to_array())
    
    def to_NED(self):
        return rotate_ENU_NED(self.to_array())
    
    def to_global(self, reference_orientation, rotation_type='quaternion'):
        return transform_coordinates(
            source_position=self.to_array(),
            reference_position=np.asarray([0,0,0]),
            reference_orientation=reference_orientation,
            transform_type='local_to_global',
            rotation_type=rotation_type)
    
    def to_local(self, reference_orientation, rotation_type='quaternion'):
        return transform_coordinates(
            source_position=self.to_array(),
            reference_position=np.asarray([0,0,0]),
            reference_orientation=reference_orientation,
            transform_type='global_to_local',
            rotation_type=rotation_type)
    
    def __call__(self):
        """
        Returns velocities as a numpy array when the object is called.
        """
        return self.to_array()

    def __repr__(self):
        return f"Velocity(vx={self.vx}, vy={self.vy}, vz={self.vz})"


class QuaternionData(BaseData):
    """
    Class for representing a quaternion, adapted for use with numpy.

    Attributes:
    - quaternion (np.ndarray): Numpy array containing [x, y, z, w].
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self._quaternion = np.array([self._validate_and_convert(x),
                                    self._validate_and_convert(y),
                                    self._validate_and_convert(z),
                                    self._validate_and_convert(w)])
        self.normalize()

    @property
    def x(self):
        return self._quaternion[0]

    @x.setter
    def x(self, value):
        self._quaternion[0] = self._validate_and_convert(value)

    @property
    def y(self):
        return self._quaternion[1]

    @y.setter
    def y(self, value):
        self._quaternion[1] = self._validate_and_convert(value)

    @property
    def z(self):
        return self._quaternion[2]

    @z.setter
    def z(self, value):
        self._quaternion[2] = self._validate_and_convert(value)

    @property
    def w(self):
        return self._quaternion[3]

    @w.setter
    def w(self, value):
        self._quaternion[3] = self._validate_and_convert(value)

    def normalize(self):
        """
        Normalizes the quaternion.
        """
        norm = np.linalg.norm(self._quaternion)
        if norm > 0:
            self._quaternion /= norm

    def to_array(self) -> np.ndarray:
        """
        Returns the quaternion as a numpy array.
        """
        return self._quaternion.copy()

    def update(self, x=None, y=None, z=None, w=None):
        """
        Updates quaternion values if new values are provided.

        :param x: X component (or None if not updated).
        :param y: Y component (or None if not updated).
        :param z: Z component (or None if not updated).
        :param w: W component (or None if not updated).
        """
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if z is not None:
            self.z = z
        if w is not None:
            self.w = w
        self.normalize()
    
    def update_from_array(self, array: np.ndarray):
        """
        Updates the quaternion from a numpy array.

        :param array: numpy array of size 4 containing [x, y, z, w].
        """
        if not isinstance(array, np.ndarray) or array.shape != (4,):
            raise ValueError("Expected numpy array of size (4,) for [x, y, z, w].")
        self._quaternion[:] = np.array([self._validate_and_convert(array[0]),
                                    self._validate_and_convert(array[1]),
                                    self._validate_and_convert(array[2]),
                                    self._validate_and_convert(array[3])])
        self.normalize()

    def to_ros_quaternion(self):
        """
        Creates a Quaternion object from geometry_msgs.msg.

        Returns:
        Quaternion: ROS Quaternion object with fields x, y, z, w.
        """
        return Quaternion(x=self.x, y=self.y, z=self.z, w=self.w)

    def to_euler(self) -> np.ndarray:
        """
        Converts quaternion to Euler angles using scipy.

        Returns:
        np.ndarray: Numpy array [roll, pitch, yaw] in radians.
        """
        # Convert quaternion to Euler angles
        euler_angles = Rotation.from_quat(self._quaternion).as_euler('zyx', degrees=False)
        return euler_angles

    def update_from_euler(self, euler: np.ndarray):
        """
        Sets the quaternion from Euler angles using scipy.

        :param euler: NumPy array with angles (roll, pitch, yaw) in radians.
        """
        if not isinstance(euler, np.ndarray) or euler.shape != (3,):
            raise ValueError("Parameter euler must be a NumPy array of shape (3,)")
        quaternion = Rotation.from_euler('xyz', euler, degrees=False).as_quat()
        self.update_from_array(quaternion)
        
    def to_ENU(self):
        return rotated_ENU_NED_quaternion(self.to_array())
    
    def to_NED(self):
        return rotated_ENU_NED_quaternion(self.to_array())
    
    def to_global(self, reference_orientation):
        return transform_orientation(
            source_orientation=self.to_array(),
            reference_orientation=reference_orientation,
            mode='quaternion',          
            transform_type='local_to_global')
    
    def to_local(self, reference_orientation):
        return transform_orientation(
            source_orientation=self.to_array(),
            reference_orientation=reference_orientation,
            mode='quaternion',          
            transform_type='global_to_local')        

    def __call__(self):
        """
        Returns the quaternion as a numpy array when the object is called.
        """
        return self.to_array()
    
    def __repr__(self):
        return f"QuaternionData(x={self.x}, y={self.y}, z={self.z}, w={self.w})"


class OrientationData:
    """
    Class for representing orientation with support for both Euler angles and quaternions.

    Attributes:
    - euler (EulerData): Euler angles object (roll, pitch, yaw).
    - quaternion (QuaternionData): Quaternion object (x, y, z, w).
    """

    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0, x=0.0, y=0.0, z=0.0, w=1.0):
        # Initialize Euler angles and quaternion
        self.euler = EulerData(roll, pitch, yaw)
        self.quaternion = QuaternionData(x, y, z, w)
        self._sync_quaternion_from_euler()

    def _sync_quaternion_from_euler(self):
        """Synchronizes the quaternion with the current Euler angles."""
        self.quaternion.update_from_euler(self.euler.to_array())

    def _sync_euler_from_quaternion(self):
        """Synchronizes Euler angles with the current quaternion."""
        self.euler.update_from_array(self.quaternion.to_euler())

    def update_from_euler(self, roll=None, pitch=None, yaw=None):
        """
        Updates orientation based on Euler angles.
        """
        self.euler.update(roll=roll, pitch=pitch, yaw=yaw)
        self._sync_quaternion_from_euler()

    def update_from_quaternion(self, x=None, y=None, z=None, w=None):
        """
        Updates orientation based on quaternion.
        """
        self.quaternion.update(x=x, y=y, z=z, w=w)
        self._sync_euler_from_quaternion()

    def update_from_euler_array(self, euler_array: np.ndarray):
        """
        Updates orientation from an Euler angle array.

        :param euler_array: numpy array of three elements [roll, pitch, yaw].
        """
        self.euler.update_from_array(euler_array)
        self._sync_quaternion_from_euler()

    def update_from_quaternion_array(self, quaternion_array: np.ndarray):
        """
        Updates orientation from a quaternion array.

        :param quaternion_array: numpy array of four elements [x, y, z, w].
        """
        self.quaternion.update_from_array(quaternion_array)
        self._sync_euler_from_quaternion()

    def normalize_quaternion(self):
        """
        Normalizes the quaternion and synchronizes Euler angles.
        """
        self.quaternion.normalize()
        self._sync_euler_from_quaternion()

    def to_dict(self):
        """
        Returns orientation as a dictionary.
        """
        return {
            "euler": self.euler.to_dict(),
            "quaternion": self.quaternion.to_dict(),
        }

    def to_ros_quaternion(self):
        """
        Returns a ROS Quaternion object.
        """
        return self.quaternion.to_ros_quaternion()

    def to_vector3(self):
        """
        Returns Euler angles as Vector3.
        """
        return self.euler.to_vector3()

    def __repr__(self):
        return f"OrientationData(euler={self.euler}, quaternion={self.quaternion})"
