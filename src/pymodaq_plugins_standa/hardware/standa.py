import platform
from qtpy import QtCore
from pylablib.devices import Standa
from serial.tools import list_ports
import time

if not hasattr(QtCore, "pyqtSignal"):
    QtCore.pyqtSignal = QtCore.Signal  # type: ignore


class StandaManager:
    def __init__(self, baudrate):
        self.devices = None
        self.connections = []
        self.interfaces = []
        self._baudrate = baudrate

    def probe_standa_ports(self):
        self.devices = {'ports': [], 'serial_numbers': []}
        ports = list_ports.comports()

        for port in ports:
            try:
                conn = Standa.Standa8SMC((port.device, self._baudrate))
                if platform.system() == 'Windows':
                    if port.serial_number is not None:
                        self.devices['ports'].append(port.device)
                        self.devices['serial_numbers'].append(port.serial_number)
                else:
                    if 'XIMC' in port.manufacturer:
                        self.devices['ports'].append(port.device)
                        self.devices['serial_numbers'].append(port.serial_number)
                conn.close()
            except Exception as e:
                pass
        return self.devices

    def connect(self, port):
        try:
            conn = Standa.Standa8SMC((port, self._baudrate))
            self.interfaces.append(conn)
            self.connections.append(port)
        except Exception as e:
            print(f"Failed to connect to Standa device at {port}: {e}")

    def close(self, port):
        try:
            if port in self.connections:
                index = self.connections.index(port)
                self.interfaces[index].close()
                del self.interfaces[index]
                del self.connections[index]
        except Exception as e:
            pass


class StandaController:
    def __init__(self, device_info):
        self.port = device_info['port']
        self.serial_number = device_info['serial_number']
        self.motor = None
        self.resolution = 0
        self.reference_position = 0
        self.favorite_positions = None
        self.power_parameters = {
            "hold_current": 0,
            "reduct_enabled": False,
            "reduct_delay": 0,
            "off_enabled": False,
            "off_delay": 0,
            "ramp_enabled": False,
            "ramp_time": 0
        }
        self.move_parameters = {
            "speed": 0,
            "accel": 0,
            "decel": 0,
            "antiplay": 0
        }

    def connect_motor(self, interface) -> None:
        """Connect to the motor using the provided interface"""
        try:
            self.motor = interface
            self.update_current_values()
        except Exception as e:
            print(f"Failed to connect to motor: {e}")
            raise

    def update_current_values(self) -> None:
        """Update current motor parameters from device"""
        if self.motor is None:
            return
        
        try:
            # Get move parameters
            move_params = self.motor.get_move_parameters()
            self.move_parameters["speed"] = move_params.speed
            self.move_parameters["accel"] = move_params.accel
            self.move_parameters["decel"] = move_params.decel
            self.move_parameters["antiplay"] = move_params.antiplay
            
            # Get power parameters
            power_params = self.motor.get_power_parameters()
            self.power_parameters["hold_current"] = power_params.hold_current
            self.power_parameters["reduct_enabled"] = power_params.reduct_enabled
            self.power_parameters["reduct_delay"] = power_params.reduct_delay
            self.power_parameters["off_enabled"] = power_params.off_enabled
            self.power_parameters["off_delay"] = power_params.off_delay
            self.power_parameters["ramp_enabled"] = power_params.ramp_enabled
            self.power_parameters["ramp_time"] = power_params.ramp_time
            
        except Exception as e:
            print(f"Failed to update current values: {e}")

    # Movement parameter properties
    @property
    def max_velocity(self):
        """Get maximum velocity (speed)"""
        return self.move_parameters["speed"]
    
    @max_velocity.setter
    def max_velocity(self, value):
        """Set maximum velocity (speed)"""
        self.motor.setup_move(speed=value)
        self.move_parameters["speed"] = value

    @property
    def max_acceleration(self):
        """Get maximum acceleration"""
        return self.move_parameters["accel"]
    
    @max_acceleration.setter
    def max_acceleration(self, value):
        """Set maximum acceleration"""
        self.motor.setup_move(accel=value, decel=value)
        self.move_parameters["accel"] = value
        self.move_parameters["decel"] = value

    # Position properties
    @property
    def actual_position(self):
        """Get current position in steps/microsteps"""
        return self.motor.get_position()

    @property
    def target_position(self):
        """Get target position (not directly available, return current)"""
        # Standa doesn't expose target position separately
        return self.actual_position

    @property
    def encoder_position(self):
        """Get encoder position"""
        return self.motor.get_encoder()

    # Velocity properties
    @property
    def actual_velocity(self):
        """Get current velocity from status"""
        status = self.motor.get_status()
        return status.speed

    @property
    def target_velocity(self):
        """Get target velocity (configured speed)"""
        return self.move_parameters["speed"]

    # Status checks
    def is_moving(self) -> bool:
        """Check if motor is currently moving"""
        return self.motor.is_moving()

    def wait_for_stop(self, timeout=None) -> None:
        """Wait until motor stops moving"""
        self.motor.wait_move(timeout=timeout)

    # Motion control methods
    def set_reference_position(self, position=0) -> None:
        """Set current position as reference"""
        self.stop()
        time.sleep(0.1)  # Brief delay to ensure stop
        self.motor.set_position_reference(position)
        self.reference_position = position

    def move_to(self, position) -> None:
        """Move to absolute position"""
        self.motor.move_to(position)

    def move_by(self, distance) -> None:
        """Move by relative distance"""
        self.motor.move_by(distance)

    def move_to_reference(self) -> None:
        """Move to reference position (zero)"""
        self.motor.move_to(self.reference_position)

    def jog(self, direction: int) -> None:
        """
        Start continuous movement in given direction
        direction: 1 for positive, -1 for negative
        """
        self.motor.jog("+" if direction > 0 else "-")

    def home(self, sync=True, timeout=30.0) -> None:
        """Execute homing routine"""
        self.motor.home(sync=sync, timeout=timeout)

    def stop(self, immediate=False) -> None:
        """
        Stop motor movement
        immediate: if True, stop immediately; if False, decelerate normally
        """
        self.motor.stop(immediate=immediate)

    def power_off(self, stop="soft") -> None:
        """
        Turn off motor power
        stop: "soft", "immediate", or "none"
        """
        self.motor.power_off(stop=stop)

    # Parameter configuration
    def setup_move_parameters(self, speed=None, accel=None, decel=None, antiplay=None):
        """Configure movement parameters"""
        result = self.motor.setup_move(speed=speed, accel=accel, decel=decel, antiplay=antiplay)
        self.update_current_values()
        return result

    def setup_power_parameters(self, hold_current=None, reduct_enabled=None, 
                               reduct_delay=None, off_enabled=None, 
                               off_delay=None, ramp_enabled=None, ramp_time=None):
        """Configure power parameters"""
        result = self.motor.setup_power(
            hold_current=hold_current,
            reduct_enabled=reduct_enabled,
            reduct_delay=reduct_delay,
            off_enabled=off_enabled,
            off_delay=off_delay,
            ramp_enabled=ramp_enabled,
            ramp_time=ramp_time
        )
        self.update_current_values()
        return result

    # Information methods
    def get_engine_type(self):
        """Get engine and driver type"""
        return self.motor.get_engine_type()

    def get_calibration(self):
        """Get stepper motor calibration"""
        return self.motor.get_stepper_motor_calibration()

    def get_full_status(self):
        """Get complete status information"""
        return self.motor.get_status()