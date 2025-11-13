import json
import inspect
from std_msgs.msg import String
from multi_drone.move_commands.base.base_commander import DroneCommander
from multi_drone.move_commands.base.base_g_code import BaseGCommand
import multi_drone.move_commands.x500.g_code as g_code_module
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class X500Commander(DroneCommander):
    """
    Commander for the X500 drone.
    """
    def __init__(self, controller: "X500Controller", timer_execution=0.1):
        super().__init__(controller)

        # Automatically register all G-code command classes from the module
        self.command_classes = {
            cls("temp").name: cls
            for _, cls in inspect.getmembers(g_code_module, inspect.isclass)
            if issubclass(cls, BaseGCommand) and cls != BaseGCommand
        }

        # Subscription to incoming JSON commands
        self.subscriber_command_json = controller.create_subscription(
            String,
            f"{controller.prefix_name}/in/command_json",
            self.command_json_callback,
            controller.qos_profile_reliable
        )
       
        # Timer for command execution loop
        self.timer_execution_commands = controller.create_timer(
            timer_execution, self._timer_execute_commands_callback
        )
       
    def _timer_execute_commands_callback(self):
        """
        ROS timer callback for executing commands.
        """
        if self.active_command:
            try:
                if self.active_command.is_complete(self.controller):
                    self.handle_completion(self.active_command)
                    self.active_command = None
            except Exception as e:
                self.controller.log_error(f"Error while checking command completion: {e}")
                self.active_command = None
        elif self.command_queue:
            self.active_command = self.command_queue.pop(0)
            self.controller.log_info(f"Executing command: {self.active_command.name}")
            self.active_command.safe_execute(self.controller)

    def command_json_callback(self, msg: String):
        """
        Callback for incoming ROS JSON commands.
        """
        try:
            data = json.loads(msg.data)
            self.process_incoming_command(data)
        except json.JSONDecodeError as e:
            self.controller.log_error(f"JSON decode error: {e}")

    def process_incoming_command(self, data: dict):
        """
        Process an incoming command.
        """
        command_name = data.get("name")
        if not command_name:
            self.controller.log_error("Command name missing in data.")
            return
       
        command_class: type[BaseGCommand] = self.command_classes.get(command_name)
        if not command_class:
            self.controller.log_error(f"Unknown command: {command_name}")
            return

        command = command_class.from_dict(data)
        self.command_history.append(data)
       
        if not self.check_special_command(command):
            self.add_command(command)

    def handle_completion(self, command: BaseGCommand):
        """
        Called when a command is completed.
        """
        self.controller.log_info(f"Command {command.name} completed.")

    def check_special_command(self, command: BaseGCommand) -> bool:
        """
        Handle special commands that should be executed immediately.
        Currently, G0 (rapid move) is executed synchronously.
        """
        if command.name == "G0":
            command.execute(self.controller)
            return True
        return False
