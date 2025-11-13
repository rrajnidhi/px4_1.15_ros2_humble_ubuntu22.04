from multi_drone.controllers.x500.states import LoiterState, OffboardState
from multi_drone.move_commands.base.base_g_code import BaseGCommand
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G6_Offboard(BaseGCommand):
    """
    Command to activate Offboard mode.
    """
    def __init__(self, counter: int = 0):
        """
        :param counter: Command counter.
        """
        super().__init__("G6", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Checks whether the command can be executed.
        Condition: current state must be LoiterState.
        """
        if isinstance(controller.current_state, LoiterState):
            return True
        controller.log_error("G6_Offboard: Command can only be executed from LoiterState.")
        return False

    def execute(self, controller: 'X500Controller'):
        """
        Enables Offboard mode via the parameter system.
        """
        controller.params.offboard_mode = True
        controller.params.landing = False
        # controller.log_info("G6_Offboard: Offboard mode activation request sent.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Checks whether Offboard mode has been successfully activated.
        Condition: current state must be OffboardState.
        """
        if isinstance(controller.current_state, OffboardState):
            # controller.log_info("G6_Offboard: Drone successfully switched to OffboardState.")
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
        return f"G6_Offboard(counter={self.counter}, complete={self.complete})"
