from multi_drone.move_commands.base.base_g_code import BaseGCommand
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G0_Stop(BaseGCommand):
    """
    Special command to forcibly stop the current command and halt the drone.
    """
    def __init__(self, counter: int = 0):
        super().__init__("G0", counter)
       
    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Checks whether the command can be executed.
        """
        return True

    def execute(self, controller: 'X500Controller'):
        """
        Interrupts the currently active command and clears the queue.
        """
        if controller.g_code_commander.active_command:
            controller.g_code_commander.active_command.mark_as_interrupted()
        controller.g_code_commander.clear_command_queue()
        controller.offboard_commander.update()
        controller.log_info("G0_Stop: All drone movements have been stopped.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Checks whether the drone has come to a complete stop.
        """
        current_velocity = controller.current_position.get_velocity(system='local_ENU')
        velocity_tolerance = 0.05  # Acceptable velocity error (m/s)
        if all(abs(v) <= velocity_tolerance for v in current_velocity):
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
        return f"G0_Stop(counter={self.counter}, complete={self.complete})"
