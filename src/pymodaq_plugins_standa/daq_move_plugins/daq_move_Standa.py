import time
from typing import Union, List, Dict, Tuple
from pymodaq.control_modules.move_utility_classes import (DAQ_Move_base, comon_parameters_fun,
                                                          main, DataActuatorType, DataActuator)

from pymodaq_utils.utils import ThreadCommand
from pymodaq_gui.parameter import Parameter
from pymodaq_plugins_standa.hardware.standa import StandaManager, StandaController
from qtpy import QtCore

from pymodaq.control_modules.thread_commands import ThreadStatus


class DAQ_Move_Standa(DAQ_Move_base):
    """
        * This has been tested with the Standa 8SMC4/8SMC5 stepper motor controllers
        * Tested on PyMoDAQ >5.0.6
        * Tested on Python 3.11
        * No additional drivers necessary
    """
    is_multiaxes = False
    _axis_names: Union[List[str], Dict[str, int]] = ['Axis 1']
    _controller_units: Union[str, List[str]] = 'dimensionless'  # steps for DC motor, microsteps for stepper
    data_actuator_type = DataActuatorType.DataActuator
    _epsilon: Union[float, List[float]] = 1  # Minimum step/microstep

    # Initialize communication
    manager = StandaManager()
    devices = manager.probe_standa_ports()

    params = [
                {'title': 'Device Management:', 'name': 'device_manager', 'type': 'group', 'children': [
                    {'title': 'Refresh Device List:', 'name': 'refresh_devices', 'type': 'bool_push', 'value': False},
                    {'title': 'Connected Devices:', 'name': 'connected_devices', 'type': 'list', 'limits': devices['ports']},
                    {'title': 'Selected Device:', 'name': 'selected_device', 'type': 'str', 'value': '', 'readonly': True},
                    {"title": "Device Serial Number", "name": "device_serial_number", "type": "str", "value": "", 'readonly': True}
                ]},
                {'title': 'Motor Info:', 'name': 'motor_info', 'type': 'group', 'children': [
                    {'title': 'Engine Type:', 'name': 'engine_type', 'type': 'str', 'value': '', 'readonly': True},
                    {'title': 'Driver Type:', 'name': 'driver_type', 'type': 'str', 'value': '', 'readonly': True},
                    {'title': 'Steps per Revolution:', 'name': 'steps_per_rev', 'type': 'int', 'value': 200, 'readonly': True},
                    {'title': 'Microsteps per Step:', 'name': 'usteps_per_step', 'type': 'int', 'value': 1, 'readonly': True},
                ]},
                {'title': 'Encoder Settings:', 'name': 'encoder', 'type': 'group', 'children': [
                    {'title': 'Encoder Position:', 'name': 'encoder_position', 'type': 'int', 'value': 0, 'readonly': True},
                    {'title': 'Set Encoder Reference:', 'name': 'set_encoder_reference', 'type': 'bool_push', 'value': False},
                ]},
                {'title': 'Positioning:', 'name': 'positioning', 'type': 'group', 'children': [
                    {'title': 'Set Reference Position:', 'name': 'set_reference_position', 'type': 'bool_push', 'value': False},
                    {'title': 'Home Motor:', 'name': 'home_motor', 'type': 'bool_push', 'value': False},
                ]},
                {'title': 'Motion Control:', 'name': 'motion', 'type': 'group', 'children': [
                    {'title': 'Speed:', 'name': 'speed', 'type': 'int', 'value': 128000, 'limits': [1, 512000]}, # Be careful going to maximum, respect your motor specs
                    {'title': 'Acceleration:', 'name': 'acceleration', 'type': 'int', 'value': 1000, 'limits': [1, 30000000]},
                    {'title': 'Deceleration:', 'name': 'deceleration', 'type': 'int', 'value': 1000, 'limits': [1, 30000000]},
                    {'title': 'Antiplay:', 'name': 'antiplay', 'type': 'int', 'value': 0, 'limits': [0, 10000]},
                ]},
                {'title': 'Power Settings:', 'name': 'power', 'type': 'group', 'children': [
                    {'title': 'Hold Current:', 'name': 'hold_current', 'type': 'int', 'value': 50, 'limits': [0, 255]}, # Be very careful with this value, it can lead to permanent damage to the motor if too high
                    {'title': 'Current Reduction:', 'name': 'current_reduction', 'type': 'group', 'children': [
                        {'title': 'Enable:', 'name': 'reduct_enabled', 'type': 'bool', 'value': False},
                        {'title': 'Delay (s):', 'name': 'reduct_delay', 'type': 'float', 'value': 1.0, 'limits': [0, 60]},
                    ]},
                    {'title': 'Power Off:', 'name': 'power_off_settings', 'type': 'group', 'children': [
                        {'title': 'Enable Auto Off:', 'name': 'off_enabled', 'type': 'bool', 'value': False},
                        {'title': 'Delay (s):', 'name': 'off_delay', 'type': 'float', 'value': 5.0, 'limits': [0, 60]},
                    ]},
                    {'title': 'Current Ramp:', 'name': 'current_ramp', 'type': 'group', 'children': [
                        {'title': 'Enable:', 'name': 'ramp_enabled', 'type': 'bool', 'value': False},
                        {'title': 'Time (s):', 'name': 'ramp_time', 'type': 'float', 'value': 0.5, 'limits': [0, 10]},
                    ]},
                    {'title': 'Power Off Now:', 'name': 'power_off_now', 'type': 'bool_push', 'value': False},
                ]},
        ] + comon_parameters_fun(is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)

    def ini_attributes(self):
        self.controller: StandaController = None
        self._last_poll_time = 0.0  # Used to throttle polling frequency

    def get_actuator_value(self):
        self._throttle_polling(20.0)  # wait 20 ms between polls
        pos = DataActuator(data=self.controller.actual_position)
        return self.get_position_with_scaling(pos)

    def user_condition_to_reach_target(self) -> bool:
        """Check once whether the target is reached, with safe polling for all needed values."""
        self._throttle_polling(20)
        # For Standa, we check if motor is still moving
        return not self.controller.is_moving()

    def close(self):
        """Terminate the communication protocol"""
        if self.is_master:
            port = self.controller.port
            self.controller.port = ''
            self.manager.close(port)
            print("Closed connection to device on port {}".format(port))
        self.controller = None

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        name = param.name()
        value = param.value()
        
        if name == 'refresh_devices':
            param = self.settings.child('device_manager', 'refresh_devices')
            param.setValue(False)
            param.sigValueChanged.emit(param, False)
            devices = self.manager.probe_standa_ports()
            self.settings.child('device_manager', 'connected_devices').setLimits(devices['ports'])
            
        elif name == 'speed':
            self.controller.setup_move_parameters(speed=value)
            
        elif name == 'acceleration':
            self.controller.setup_move_parameters(accel=value)
            
        elif name == 'deceleration':
            self.controller.setup_move_parameters(decel=value)
            
        elif name == 'antiplay':
            self.controller.setup_move_parameters(antiplay=value)
            
        elif name == 'set_reference_position':
            if value:
                self.controller.set_reference_position()
                param = self.settings.child('positioning', 'set_reference_position')
                param.setValue(False)
                param.sigValueChanged.emit(param, False)
                self.emit_status(ThreadCommand('Update_Status', ['Reference position set']))
                
        elif name == 'set_encoder_reference':
            if value:
                self.controller.motor.set_encoder_reference(0)
                param = self.settings.child('encoder', 'set_encoder_reference')
                param.setValue(False)
                param.sigValueChanged.emit(param, False)
                self.emit_status(ThreadCommand('Update_Status', ['Encoder reference set']))
                
        elif name == 'home_motor':
            if value:
                self.emit_status(ThreadCommand('Update_Status', ['Homing motor...']))
                try:
                    self.controller.home(sync=True, timeout=30.0)
                    self.emit_status(ThreadCommand('Update_Status', ['Homing complete']))
                except Exception as e:
                    self.emit_status(ThreadCommand('Update_Status', [f'Homing failed: {str(e)}']))
                param = self.settings.child('positioning', 'home_motor')
                param.setValue(False)
                param.sigValueChanged.emit(param, False)
                
        elif name == 'hold_current':
            self.controller.setup_power_parameters(hold_current=value)
            
        elif name == 'reduct_enabled':
            self.controller.setup_power_parameters(reduct_enabled=value)
            
        elif name == 'reduct_delay':
            self.controller.setup_power_parameters(reduct_delay=value)
            
        elif name == 'off_enabled':
            self.controller.setup_power_parameters(off_enabled=value)
            
        elif name == 'off_delay':
            self.controller.setup_power_parameters(off_delay=value)
            
        elif name == 'ramp_enabled':
            self.controller.setup_power_parameters(ramp_enabled=value)
            
        elif name == 'ramp_time':
            self.controller.setup_power_parameters(ramp_time=value)
            
        elif name == 'power_off_now':
            if value:
                self.controller.power_off(stop="soft")
                param = self.settings.child('power', 'power_off_now')
                param.setValue(False)
                param.sigValueChanged.emit(param, False)
                self.emit_status(ThreadCommand('Update_Status', ['Motor power off']))
                
        elif name == 'use_scaling':
            # Just use this current value in UI
            self.poll_moving()

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        # Always get a fresh list on device initialization
        devices = self.manager.probe_standa_ports()
        self.settings.child('device_manager', 'connected_devices').setLimits(devices['ports'])
        index = devices['ports'].index(self.settings.child('device_manager', 'connected_devices').value())
        device_info = {'port': devices['ports'][index], 'serial_number': devices['serial_numbers'][index]}

        self.ini_stage_init(slave_controller=controller)  # will be useful when controller is slave

        if self.is_master:  # is needed when controller is master
            self.controller = StandaController(device_info)
            initialized = True
        else:
            self.controller = controller
            initialized = True
        
        # Establish connection
        self.manager.connect(self.controller.port)
        self.controller.connect_motor(self.manager.interfaces[self.manager.connections.index(self.controller.port)])
        
        # Update UI with device info
        self.settings.child('device_manager', 'selected_device').setValue(self.controller.port)
        self.settings.child('device_manager', 'device_serial_number').setValue(self.controller.serial_number)

        # Get motor info
        engine_info = self.controller.get_engine_type()
        self.settings.child('motor_info', 'engine_type').setValue(engine_info.engine)
        self.settings.child('motor_info', 'driver_type').setValue(engine_info.driver)
        
        calib_info = self.controller.get_calibration()
        self.settings.child('motor_info', 'steps_per_rev').setValue(calib_info.steps_per_rev)
        self.settings.child('motor_info', 'usteps_per_step').setValue(calib_info.usteps_per_step)

        # Apply motion control settings
        self.controller.setup_move_parameters(
            speed=self.settings.child('motion', 'speed').value(),
            accel=self.settings.child('motion', 'acceleration').value(),
            decel=self.settings.child('motion', 'deceleration').value(),
            antiplay=self.settings.child('motion', 'antiplay').value()
        )

        # Apply power settings
        self.controller.setup_power_parameters(
            hold_current=self.settings.child('power', 'hold_current').value(),
            reduct_enabled=self.settings.child('power', 'current_reduction', 'reduct_enabled').value(),
            reduct_delay=self.settings.child('power', 'current_reduction', 'reduct_delay').value(),
            off_enabled=self.settings.child('power', 'power_off_settings', 'off_enabled').value(),
            off_delay=self.settings.child('power', 'power_off_settings', 'off_delay').value(),
            ramp_enabled=self.settings.child('power', 'current_ramp', 'ramp_enabled').value(),
            ramp_time=self.settings.child('power', 'current_ramp', 'ramp_time').value()
        )

        # This setting never relevant for single axis
        self.settings.child('multiaxes').hide()

        # Set initial timeout
        self.settings.child('timeout').setValue(100)

        info = f"Standa actuator on port {self.controller.port} initialized"
        return info, initialized

    def move_abs(self, position: DataActuator):
        """ Move the actuator to the absolute target defined by position

        Parameters
        ----------
        position: (float) value of the absolute target positioning
        """
        position = self.check_bound(position)  # if user checked bounds, the defined bounds are applied here
        self.target_value = position
        position = self.set_position_with_scaling(position)  # apply scaling if the user specified one
        self.controller.move_to(int(round(position.value())))
        
        self.emit_status(ThreadCommand('Update_Status', ['Moving to absolute position: {}'.format(
            self.get_position_with_scaling(position).value())]))

    def move_rel(self, position: DataActuator):
        """ Move the actuator to the relative target actuator value defined by position

        Parameters
        ----------
        position: (float) value of the relative target positioning
        """
        position = self.check_bound(self.current_position + position) - self.current_position
        self.target_value = position + self.current_position
        position = self.set_position_relative_with_scaling(position)
        self.controller.move_by(int(round(position.value())))

        self.emit_status(ThreadCommand('Update_Status', ['Moving by: {}'.format(
            self.get_position_with_scaling(position).value())]))

    def move_home(self):
        """Call the reference method of the controller"""
        self.target_value = self.controller.reference_position
        self.controller.move_to_reference()
        self.emit_status(ThreadCommand('Update_Status', ['Moving to reference position']))
        self.poll_moving()

    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""
        self.controller.stop(immediate=False)
        self.move_done()
        self.emit_status(ThreadCommand('Update_Status', ['Stop motion']))

    def _throttle_polling(self, min_interval_ms: float = 10.0):
        """Ensures that polls to the hardware are not too frequent."""
        now = time.perf_counter()
        elapsed_ms = (now - self._last_poll_time) * 1000
        remaining = min_interval_ms - elapsed_ms
        if remaining > 0:
            QtCore.QThread.msleep(int(remaining))
        self._last_poll_time = time.perf_counter()


if __name__ == '__main__':
    main(__file__, init=False)