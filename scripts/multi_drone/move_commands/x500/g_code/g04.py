from multi_drone.controllers.x500.states import LoiterState, OffboardState, LandingState
from multi_drone.move_commands.base.base_g_code import BaseGCommand
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G4_Land(BaseGCommand):
    """
    Command for automatic drone landing.
    """
    def __init__(self, counter: int = 0):
        """
        :param counter: Command counter.
        """
        super().__init__("G4", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Checks whether the command can be executed.
        Condition: drone must be in LoiterState or OffboardState.
        """
        if isinstance(controller.current_state, (LoiterState, OffboardState)):
            return True
        controller.log_error(
            "G4_Land: Command can only be executed from LoiterState or OffboardState."
        )
        return False

    def execute(self, controller: 'X500Controller'):
        """
        Initiates the drone landing process.
        """
        controller.params.offboard_mode = False
        controller.params.landing = True
        # controller.log_info("G4_Land: Landing request sent.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Checks whether the landing command is complete.
        Condition: current state must be LandingState.
        """
        if isinstance(controller.current_state, LandingState):
            # controller.log_info("G4_Land: Drone is now in LandingState. Landing initiated.")
            self.complete_command()
        return self._check_finish()

    def to_dict(self) -> dict:
        """
        Serializes the command into a dictionary.
        """
        return super().to_dict()

    @classmethod
    def from_dict(cls, data: dict):
        """
        Deserializes the command from a dictionary.
        """
        return cls(counter=data.get("counter", 0))

    def __repr__(self):
        """
        String representation of the command.
        """
        return f"G4_Land(counter={self.counter}, complete={self.complete})"
