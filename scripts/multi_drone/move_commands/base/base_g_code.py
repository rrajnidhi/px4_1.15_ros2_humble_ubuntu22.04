# base_command.py
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multi_drone.controllers.base.base_controller import BaseDroneController


class BaseGCommand(ABC):
    def __init__(self, name, counter=0, current_step=0):
        self.name = name
        self.counter = counter
        self.complete = False
        self.interrupt = False
        self.current_step = current_step

    @abstractmethod
    def execute(self, controller):
        """
        Executes the current command using the provided controller.
        :param controller: Controller instance used to send commands to the drone.
        """
        pass

    @abstractmethod
    def is_complete(self, controller: 'BaseDroneController') -> bool:
        """
        Checks whether the command is complete.
        :param controller: Controller instance used to check drone state.
        :return: True if the command is complete, otherwise False.
        """
        pass

    @abstractmethod
    def can_execute(self, controller: 'BaseDroneController') -> bool:
        """
        Checks whether the command can be executed.
        :param controller: Controller instance used to verify execution conditions.
        :return: True if the command can be executed, otherwise False.
        """
        pass

    def safe_execute(self, controller: 'BaseDroneController'):
        """
        Checks completion and executability before calling execute.
        :param controller: Controller instance.
        """
        if self._check_finish():
            # controller.log_info(f"Command {self.name} is already complete.")
            return
        if not self.can_execute(controller):
            # controller.log_error(f"Command {self.name} cannot be executed in the current state.")
            self.mark_as_interrupted()
            return
        self.execute(controller)
       
    def _check_finish(self):
        if self.complete or self.interrupt:
            return True
        else:
            return False
   
    def mark_as_interrupted(self):
        """
        Sets the flag indicating that the command should be interrupted.
        """
        self.interrupt = True
       
    def complete_command(self):
        """
        Sets the flag indicating that the command is complete.
        """
        self.complete = True

    def to_dict(self) -> dict:
        """
        Returns a dictionary with command parameters.
        :return: Dictionary containing command parameters.
        """
        return {
            "name": self.name,
            "counter": self.counter,
            "current_step": self.current_step
        }

    @classmethod
    def from_dict(cls, data: dict):
        """
        Creates a command instance from a dictionary.
        :param data: Dictionary with command parameters.
        :return: Command instance.
        """
        return cls(
            name=data["name"],
            counter=data["counter"],
            current_step=data["current_step"]
        )

    def __repr__(self):
        """
        String representation of the command for debugging.
        """
        return f"{self.__class__.__name__}(name={self.name}, counter={self.counter}, complete={self.complete})"
