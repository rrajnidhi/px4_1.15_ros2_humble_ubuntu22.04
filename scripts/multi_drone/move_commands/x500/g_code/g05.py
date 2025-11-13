from multi_drone.controllers.x500.states import OffboardState, LoiterState
from multi_drone.move_commands.base.base_g_code import BaseGCommand
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G5_Loiter(BaseGCommand):
    """
    Command to switch the drone to Loiter (hover) mode.
    """
    def __init__(self, counter: int = 0):
        """
        :param counter: Command counter.
        """
        super().__init__("G5", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Checks whether the command can be executed.
        Condition: current state must be OffboardState.
        """
        if not isinstance(controller.current_state, OffboardState):
            controller.log_error("G5_Loiter: Command can only be executed from OffboardState.")
            return False
        return True

    def execute(self, controller: 'X500Controller'):
        """
        Switches the drone to Loiter mode by setting the appropriate parameters.
        """
        controller.params.offboard_mode = False
        controller.params.landing = False
        # controller.log_info("G5_Loiter: Transition to Loiter mode initiated.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Checks whether the drone has successfully entered LoiterState.
        """
        if isinstance(controller.current_state, LoiterState):
            # controller.log_info("G5_Loiter: Drone successfully transitioned to LoiterState.")
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
        return f"G5_Loiter(counter={self.counter}, complete={self.complete})"
