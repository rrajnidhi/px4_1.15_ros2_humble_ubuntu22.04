from abc import ABC, abstractmethod
from time import sleep
from px4_msgs.msg import VehicleStatus
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500_base import X500BaseController


class DroneState(ABC):
    """
    Base class for all drone states.
    """
    def __init__(self, controller: "X500BaseController"):
        self.controller = controller
        self.params = controller.params
        self.counter = 0

    @abstractmethod
    def enter(self):
        """
        Method called when entering the state.
        """
        pass

    @abstractmethod
    def handle(self):
        """
        Method called to process the logic of the current state.
        """
        pass

    @abstractmethod
    def exit(self):
        """
        Method called when exiting the state.
        """
        pass


class IdleState(DroneState):
    """
    Idle (waiting) state.
    """
    def enter(self):
        self.controller.log_info("Entering IDLE state.")
        self.controller.params.reset()

    def handle(self):
        if self.params.flight_check and self.params.arming:
            self.controller.set_state("ARMING")

    def exit(self):
        self.controller.log_info("Exiting IDLE state.")


class ArmingState(DroneState):
    """
    Drone arming state.
    """
    def enter(self):
        self.controller.log_info("Entering ARMING state.")
        self.controller.set_home_to_current_position()
        self.controller.calibrate_gyroscope()
        self.controller.calibrate_gyroscope()
        self.controller.reset_ekf()

    def handle(self):
        if not self.params.flight_check:
            self.controller.set_state("IDLE")
        elif not self.params.arming:
            self.controller.set_state("DISARM")
        elif (self.params.arm_state == VehicleStatus.ARMING_STATE_ARMED
              and self.counter > 10 and self.params.takeoff):
            self.controller.set_state("TAKEOFF")
        self.controller.arm()
        self.counter += 1

    def exit(self):
        self.controller.log_info("Exiting ARMING state.")


class TakeoffState(DroneState):
    """
    Takeoff state.
    """
    def enter(self):
        self.controller.log_info("Entering TAKEOFF state.")
        self.controller.takeoff()
        # TODO: Need to check where it's better to set the home position

    def handle(self):
        if not self.params.flight_check:
            self.controller.set_state("IDLE")
        elif self.params.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            self.controller.set_state("LOITER")
        self.controller.arm()

    def exit(self):
        self.controller.log_info("Exiting TAKEOFF state.")


class LoiterState(DroneState):
    """
    Position hold (LOITER) state.
    """
    def enter(self):
        self.controller.log_info("Entering LOITER state.")
        self.controller.enable_loiter_mode()

    def handle(self):
        if not self.params.flight_check:
            self.controller.set_state("IDLE")
        elif self.params.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            if self.params.offboard_mode:
                self.controller.set_state("OFFBOARD")
            elif self.params.landing:
                self.controller.set_state("LANDING")
        self.controller.arm()

    def exit(self):
        self.controller.log_info("Exiting LOITER state.")


class OffboardState(DroneState):
    """
    Offboard control mode state.
    """
    def enter(self):
        self.controller.log_info("Entering OFFBOARD state.")
        self.controller.offboard_commander.activate()
        self.controller.enable_offboard_mode()

    def handle(self):
        if not self.params.flight_check or self.params.failsafe:
            self.controller.set_state("IDLE")
        elif not self.params.offboard_mode:
            self.controller.set_state("LOITER")
           
        if self.counter < 10:
            self.counter += 1
            self.controller.enable_offboard_mode()

    def exit(self):
        self.controller.log_info("Exiting OFFBOARD state.")
        self.controller.offboard_commander.desactivate()


class LandingState(DroneState):
    """
    Landing state.
    """
    def enter(self):
        self.controller.log_info("Entering LANDING state.")

    def handle(self):
        self.controller.arm()
        self.controller.land()
        if (self.params.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                and not self.params.arming):
            self.controller.set_state("DISARM")

    def exit(self):
        self.controller.log_info("Exiting LANDING state.")


class DisarmState(DroneState):
    """
    Disarming state.
    """
    def enter(self):
        self.controller.log_info("Entering DISARM state.")
        self.controller.disarm()

    def handle(self):
        if self.controller.params.arm_state == VehicleStatus.ARMING_STATE_STANDBY:
            self.controller.set_state("IDLE")

    def exit(self):
        self.controller.log_info("Exiting DISARM state.")
