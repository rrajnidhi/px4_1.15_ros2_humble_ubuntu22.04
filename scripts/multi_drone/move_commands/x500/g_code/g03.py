from multi_drone.controllers.x500.states import ArmingState, LoiterState
from multi_drone.move_commands.base.base_g_code import BaseGCommand
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G3_Takeoff(BaseGCommand):
    """
    Command for automatic drone takeoff.
    """
    def __init__(self, counter: int = 0, altitude: float = 2.5):
        """
        :param counter: Command counter.
        :param altitude: Target takeoff altitude (in meters).
        """
        super().__init__("G3", counter)
        self.altitude = altitude

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Checks whether the command can be executed.
        Condition: drone must be in ArmingState.
        """
        if not isinstance(controller.current_state, ArmingState):
            controller.log_error("G3_Takeoff: Command can only be executed from ArmingState.")
            return False
        return True

    def execute(self, controller: 'X500Controller'):
        """
        Sets the takeoff target to the specified altitude.
        """
        controller.params.landing = False
        controller.params.takeoff = True
        # controller.log_info(f"G3_Takeoff: Takeoff initiated to altitude {self.altitude} m.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Checks whether takeoff was successful.
        Condition: current state must be LoiterState.
        """
        if isinstance(controller.current_state, LoiterState):
            # controller.log_info("G3_Takeoff: Drone successfully transitioned to LoiterState.")
            self.complete_command()
        return self._check_finish()

    def to_dict(self) -> dict:
        """
        Serializes the command into a dictionary.
        """
        base_dict = super().to_dict()
        base_dict.update({"altitude": self.altitude})
        return base_dict

    @classmethod
    def from_dict(cls, data: dict):
        """
        Deserializes the command from a dictionary.
        """
        return cls(
            counter=data.get("counter", 0),
            altitude=data.get("altitude", 2.5)
        )

    def __repr__(self):
        """
        String representation of the command.
        """
        return f"G3_Takeoff(counter={self.counter}, altitude={self.altitude}, complete={self.complete})"
