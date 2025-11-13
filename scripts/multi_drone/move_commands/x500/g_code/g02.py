from multi_drone.controllers.x500.states import ArmingState, LandingState, IdleState
from multi_drone.move_commands.base.base_g_code import BaseGCommand
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G2_Disarm(BaseGCommand):
    """
    Command to disarm the drone (disable motors).
    """
    def __init__(self, counter: int = 0):
        """
        :param counter: Command counter.
        """
        super().__init__("G2", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Checks whether the command can be executed.
        Condition: current state must be ArmingState or LandingState.
        """
        if isinstance(controller.current_state, (ArmingState, LandingState)):
            return True
        controller.log_error(
            "G2_Disarm: Command can only be executed from ArmingState or LandingState."
        )
        return False

    def execute(self, controller: 'X500Controller'):
        """
        Performs drone disarming.
        """
        controller.params.arming = False
        # controller.log_info("G2_Disarm: Disarming request sent.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Checks whether disarming was successful.
        Condition: current state must be IdleState.
        """
        if isinstance(controller.current_state, IdleState):
            # controller.log_info("G2_Disarm: Drone successfully disarmed (IdleState).")
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
        return f"G2_Disarm(counter={self.counter}, complete={self.complete})"
