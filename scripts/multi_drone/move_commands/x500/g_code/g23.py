import numpy as np
from multi_drone.move_commands.x500.g_code.base_move_command import BaseMoveGCommand
from typing import TYPE_CHECKING, List, Literal, Optional

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G23_Orbit(BaseMoveGCommand):
    """
    Command for flying the drone in a circular orbit around a specified center point.
    """
    def __init__(
        self,
        counter: int = 0,
        center_point: Optional[List[float]] = None,
        radius: float = 1.0,
        angular_velocity: float = 0.1,  # rad/s
        orbit_direction: Literal["CW", "CCW"] = "CW",  # Clockwise, Counterclockwise
        yaw_mode: Literal["fixed", "facing_center"] = "facing_center",
        duration: Optional[float] = None,  # Orbit duration (seconds)
        velocity: Optional[float] = None,  # Linear speed (m/s), overrides angular_velocity if provided
        coordinate_system: Literal[
            "local_NED", "local_ENU", "global_ENU", "global_NED"
        ] = "global_ENU",
        current_step: int = 0
    ):
        """
        :param counter: Command counter.
        :param center_point: Orbit center [x, y, z].
        :param radius: Orbit radius (meters).
        :param angular_velocity: Angular velocity (rad/s).
        :param orbit_direction: Orbit direction ("CW" = clockwise, "CCW" = counterclockwise).
        :param yaw_mode: Yaw behavior ("fixed" = constant yaw, "facing_center" = always face center).
        :param duration: Total execution time of the orbit (seconds). If None, one full circle.
        :param velocity: Linear speed (m/s). If provided, overrides angular_velocity (v = r * ω).
        :param coordinate_system: Coordinate system used.
        """
        super().__init__("G23", counter, coordinate_system, current_step)
        self.center_point = np.array(center_point) if center_point else None
        self.radius = radius
        self.angular_velocity = angular_velocity
        self.orbit_direction = orbit_direction
        self.yaw_mode = yaw_mode
        self.duration = duration
        self.velocity = velocity if velocity else radius * angular_velocity
        self.update_targets_positions()

    def update_targets_positions(self):
        """
        Generates the list of waypoints forming a circular orbit around the center point.
        """
        if self.center_point is None:
            return
       
        self.targets_positions.clear()

        # Determine number of points based on duration or default to full circle
        if self.duration:
            total_angle = self.angular_velocity * self.duration
            num_points = max(2, int(total_angle / (2 * np.pi / 100)))  # ~100 points per full circle
        else:
            num_points = 100  # Default: 100 points for one full orbit

        angles = np.linspace(
            0,
            2 * np.pi if not self.duration else self.angular_velocity * self.duration,
            num_points
        )

        # Reverse direction for clockwise orbit
        if self.orbit_direction == "CW":
            angles = -angles

        for angle in angles:
            x = self.center_point[0] + self.radius * np.cos(angle)
            y = self.center_point[1] + self.radius * np.sin(angle)
            z = self.center_point[2]  # Constant altitude

            # Compute yaw: face center if requested
            if self.yaw_mode == "facing_center":
                yaw = np.arctan2(self.center_point[1] - y, self.center_point[0] - x)
            else:
                yaw = None

            self.targets_positions.append(
                self.Position(x=x, y=y, z=z, velocity=self.velocity, yaw=yaw)
            )

    def to_dict(self) -> dict:
        """
        Serializes the command into a dictionary.
        """
        base_dict = super().to_dict()
        base_dict.update({
            "center_point": self.center_point.tolist() if self.center_point is not None else None,
            "radius": self.radius,
            "angular_velocity": self.angular_velocity,
            "orbit_direction": self.orbit_direction,
            "yaw_mode": self.yaw_mode,
            "duration": self.duration,
            "velocity": self.velocity,
            "coordinate_system": self.coordinate_system
        })
        return base_dict

    @classmethod
    def from_dict(cls, data: dict):
        """
        Deserializes the command from a dictionary.
        """
        return cls(
            counter=data.get("counter", 0),
            center_point=data.get("center_point"),
            radius=data.get("radius", 1.0),
            angular_velocity=data.get("angular_velocity", 0.1),
            orbit_direction=data.get("orbit_direction", "CW"),
            yaw_mode=data.get("yaw_mode", "facing_center"),
            duration=data.get("duration"),
            velocity=data.get("velocity"),
            coordinate_system=data.get("coordinate_system", "global_ENU"),
            current_step=data.get("current_step", 0)
        )

    def __repr__(self):
        """
        String representation of the command.
        """
        return (
            f"G23_Orbit(counter={self.counter}, center_point={self.center_point}, "
            f"radius={self.radius}, angular_velocity={self.angular_velocity}, "
            f"orbit_direction={self.orbit_direction}, yaw_mode={self.yaw_mode}, "
            f"duration={self.duration}, velocity={self.velocity}, complete={self.complete})"
        )
