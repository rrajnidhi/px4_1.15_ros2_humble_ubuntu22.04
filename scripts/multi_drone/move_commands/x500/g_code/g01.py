from multi_drone.controllers.x500.states import IdleState, ArmingState
from multi_drone.move_commands.base.base_g_code import BaseGCommand
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G1_Arm(BaseGCommand):
    """
    Command to arm the drone (enable motors).
    """
    def __init__(self, counter: int = 0):
        super().__init__("G1", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Checks whether the command can be executed.
        Condition: the drone must be in Idle state.
        """
        if not isinstance(controller.current_state, IdleState):
            controller.log_error("G1_Arm: Cannot execute command. Drone is not in Idle state.")
            return False
        return True

    def execute(self, controller: 'X500Controller'):
        """
        Performs drone arming by requesting a state change.
        """
        controller.params.arming = True
        # controller.log_info("G1_Arm: Arming request sent.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Checks whether arming was successful.
        Condition: current state must be ArmingState.
        """
        if isinstance(controller.current_state, ArmingState):
            # controller.log_info("G1_Arm: Drone successfully armed and now in ArmingState.")
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
        return f"G1_Arm(counter={self.counter}, complete={self.complete})"
