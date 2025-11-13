import numpy as np
from multi_drone.utils.geometry import calculate_distance
from multi_drone.move_commands.x500.g_code.base_move_command import BaseMoveGCommand
from typing import TYPE_CHECKING, List, Optional, Literal

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G21_LinearMove(BaseMoveGCommand):
    """
    Command for linear movement of the drone between two points.
    """
    def __init__(
        self,
        counter: int = 0,
        start_point: Optional[List[float]] = None,
        end_point: Optional[List[float]] = None,
        velocity: float = 1.0,
        yaw: Optional[float] = None,
        coordinate_system: Literal[
            "local_NED", "local_ENU", 'global_ENU', 'global_NED'
        ] = 'global_ENU',
        current_step: int = 0
    ):
        """
        :param counter: Command counter.
        :param start_point: Starting point [x, y, z].
        :param end_point: Ending point [x, y, z].
        :param velocity: Movement speed (m/s).
        :param yaw: Yaw orientation (radians).
        :param coordinate_system: Coordinate system used for the movement.
        """
        super().__init__("G21", counter, coordinate_system, current_step)
        self.start_point = np.array(start_point) if start_point else None
        self.end_point = np.array(end_point) if end_point else None
        self.velocity = velocity
        self.yaw = yaw
        self.update_targets_positions()

    def update_targets_positions(self):
        """
        Builds the list of target positions (start_point to end_point).
        """
        self.targets_positions.clear()
        # Add start point
        if self.start_point is not None:
            self.targets_positions.append(
                self.Position(
                    x=self.start_point[0],
                    y=self.start_point[1],
                    z=self.start_point[2],
                    velocity=self.velocity,
                    yaw=self.yaw
                )
            )
        # Add end point
        if self.end_point is not None:
            self.targets_positions.append(
                self.Position(
                    x=self.end_point[0],
                    y=self.end_point[1],
                    z=self.end_point[2],
                    velocity=self.velocity,
                    yaw=self.yaw
                )
            )

    def to_dict(self) -> dict:
        """
        Serializes the command into a dictionary.
        """
        base_dict = super().to_dict()
        base_dict.update({
            "start_point": self.start_point.tolist() if self.start_point is not None else None,
            "end_point": self.end_point.tolist() if self.end_point is not None else None,
            "velocity": self.velocity,
            "yaw": self.yaw,
            "coordinate_system": self.coordinate_system,
        })
        return base_dict

    @classmethod
    def from_dict(cls, data: dict):
        """
        Deserializes the command from a dictionary.
        """
        instance = cls(
            counter=data.get("counter", 0),
            start_point=data.get("start_point"),
            end_point=data.get("end_point"),
            velocity=data.get("velocity", 1.0),
            yaw=data.get("yaw"),
            coordinate_system=data.get("coordinate_system", 'global_ENU'),
            current_step=data.get("current_step", 0)
        )
        return instance

    def __repr__(self):
        """
        String representation of the command.
        """
        return (
            f"G21_LinearMove(counter={self.counter}, start_point={self.start_point}, "
            f"end_point={self.end_point}, velocity={self.velocity}, yaw={self.yaw}, "
            f"complete={self.complete})"
        )
