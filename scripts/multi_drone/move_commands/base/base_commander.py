from abc import ABC, abstractmethod
from typing import Type
from threading import Lock
from multi_drone.controllers.base.base_controller import BaseDroneController
from multi_drone.move_commands.base.base_g_code import BaseGCommand


class DroneCommander(ABC):
    """
    Base class for commanders that manage drone commands.
    """
    def __init__(self, controller: BaseDroneController):
        self.controller = controller
        self.command_classes = {}
        self.command_queue = []
        self.command_history = []
        self.lock = Lock()
        self.active_command: BaseGCommand = None

    @abstractmethod
    def process_incoming_command(self, data: dict):
        """
        Method for processing an incoming command.
        :param data: Dictionary containing command data.
        """
        pass

    def add_command(self, command: BaseGCommand):
        """
        Adds a command to the queue.
        :param command: Command object.
        """
        with self.lock:
            self.command_queue.append(command)

    @abstractmethod
    def handle_completion(self, command):
        """
        Method called when a command is completed.
        """
        pass

    def clear_command_queue(self):
        """
        Clears all commands from the queue.
        """
        with self.lock:
            self.command_queue.clear()
       
    def add_command_class(self, name: str, class_type: Type[object]) -> None:
        """
        Adds a key and class to the dictionary.
        :param name: Name to use as the key.
        :param class_type: Python class to store in the dictionary.
        """
        if name in self.command_classes:
            raise ValueError(f"Class with name '{name}' already exists in the dictionary.")
        self.command_classes[name] = class_type
