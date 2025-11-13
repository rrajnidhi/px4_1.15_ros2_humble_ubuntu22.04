import numpy as np
from multi_drone.move_commands.x500.g_code.base_move_command import BaseMoveGCommand
from typing import TYPE_CHECKING, List, Literal, Optional

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G22_CircularTrajectory(BaseMoveGCommand):
    """
    Command to move the drone along a circular arc with specified parameters.
    """
    def __init__(
        self,
        counter: int = 0,
        start_point: Optional[List[float]] = None,
        end_point: Optional[List[float]] = None,
        radius: float = 1.0,
        direction: Literal["CW", "CCW"] = "CW",  # Clockwise, Counterclockwise
        points_count: int = 10,
        velocity: float = 1.0,
        yaw: Optional[float] = None,
        coordinate_system: Literal[
            "local_NED", "local_ENU", "global_ENU", "global_NED"
        ] = "global_ENU",
        current_step: int = 0
    ):
        """
        :param counter: Command counter.
        :param start_point: Starting point [x, y, z].
        :param end_point: Ending point [x, y, z].
        :param radius: Arc radius (meters).
        :param direction: Direction ("CW" = clockwise, "CCW" = counterclockwise).
        :param points_count: Number of waypoints along the arc.
        :param velocity: Movement speed (m/s).
        :param yaw: Yaw orientation (radians).
        :param coordinate_system: Coordinate system used.
        """
        super().__init__("G22", counter, coordinate_system, current_step)
        self.start_point = np.array(start_point) if start_point else None
        self.end_point = np.array(end_point) if end_point else None
        self.radius = radius
        self.direction = direction
        self.points_count = points_count
        self.velocity = velocity
        self.yaw = yaw
        self.update_targets_positions()

    def update_targets_positions(self):
        """
        Generates the list of waypoints along the circular arc from start_point to end_point.
        """
        self.targets_positions.clear()
        if self.start_point is None or self.end_point is None:
            return

        # Arc center
        midpoint = (self.start_point + self.end_point) / 2
        direction_vector = self.end_point - self.start_point
        direction_vector = direction_vector / np.linalg.norm(direction_vector)

        # Normal vector (perpendicular to the arc direction)
        if self.direction == "CW":
            normal_vector = np.array([-direction_vector[1], direction_vector[0], 0])
        else:  # "CCW"
            normal_vector = np.array([direction_vector[1], -direction_vector[0], 0])

        # Arc center accounting for radius
        center = midpoint + normal_vector * self.radius

        # Start and end angles relative to the center
        start_angle = np.arctan2(self.start_point[1] - center[1], self.start_point[0] - center[0])
        end_angle = np.arctan2(self.end_point[1] - center[1], self.end_point[0] - center[0])

        # Handle angle wrapping for correct direction
        if self.direction == "CW" and end_angle > start_angle:
            end_angle -= 2 * np.pi
        elif self.direction == "CCW" and start_angle > end_angle:
            end_angle += 2 * np.pi

        # Generate points along the arc
        angles = np.linspace(start_angle, end_angle, self.points_count)
        for angle in angles:
            x = center[0] + self.radius * np.cos(angle)
            y = center[1] + self.radius * np.sin(angle)
            z = self.start_point[2]  # Keep constant altitude
            self.targets_positions.append(
                self.Position(x=x, y=y, z=z, velocity=self.velocity, yaw=self.yaw)
            )

    def to_dict(self) -> dict:
        """
        Serializes the command into a dictionary.
        """
        base_dict = super().to_dict()
        base_dict.update({
            "start_point": self.start_point.tolist() if self.start_point is not None else None,
            "end_point": self.end_point.tolist() if self.end_point is not None else None,
            "radius": self.radius,
            "direction": self.direction,
            "points_count": self.points_count,
            "velocity": self.velocity,
            "yaw": self.yaw,
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
            start_point=data.get("start_point"),
            end_point=data.get("end_point"),
            radius=data.get("radius", 1.0),
            direction=data.get("direction", "CW"),
            points_count=data.get("points_count", 10),
            velocity=data.get("velocity", 1.0),
            yaw=data.get("yaw"),
            coordinate_system=data.get("coordinate_system", "global_ENU"),
            current_step=data.get("current_step", 0)
        )

    def __repr__(self):
        """
        String representation of the command.
        """
        return (
            f"G22_CircularTrajectory(counter={self.counter}, start_point={self.start_point}, "
            f"end_point={self.end_point}, radius={self.radius}, direction={self.direction}, "
            f"points_count={self.points_count}, velocity={self.velocity}, yaw={self.yaw}, "
            f"complete={self.complete})"
        )
