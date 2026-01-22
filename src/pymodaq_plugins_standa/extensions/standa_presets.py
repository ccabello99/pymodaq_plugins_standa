from qtpy import QtWidgets, QtCore
from pathlib import Path
from typing import Optional
import json

from pymodaq_gui import utils as gutils
from pymodaq_utils.config import Config
from pymodaq_utils.logger import set_logger, get_module_name

from pymodaq.utils.config import Config as PyMoConfig
from pymodaq.extensions.utils import CustomExt

from pymodaq_gui.parameter import utils as putils

logger = set_logger(get_module_name(__file__))

config_utils = Config()
config_pymodaq = PyMoConfig()

EXTENSION_NAME = 'Standa Presets'
CLASS_NAME = 'StandaPresets'

class StandaPresetsConfig(Config):
    config_name = 'standa_presets'
    config_template_path = None

standa_config = StandaPresetsConfig()


class StandaPresets(CustomExt):
    """
    PyMoDAQ Extension for Standa motor preset positions.
    
    This extension allows you to define up to 4 preset positions and move
    to them with a single button click.
    """
    settings_name = 'StandaPresetsSettings'
    
    params = [
        {'title': 'Actuator Settings', 'name': 'actuator_settings', 'type': 'group', 'expanded': True,
         'children': [
             {'title': 'Actuator Name:', 'name': 'actuator_name', 'type': 'str', 'value': 'Standa'},
             {'title': 'Get Current Position', 'name': 'get_current_pos', 'type': 'action'},
             {'title': 'Current Position:', 'name': 'current_position', 'type': 'float', 
              'value': 0.0, 'readonly': True},
         ]},
        {'title': 'Preset Positions', 'name': 'presets', 'type': 'group', 'expanded': True,
         'children': [
             {'title': 'Preset 1', 'name': 'preset1', 'type': 'group', 'children': [
                 {'title': 'Enabled:', 'name': 'enabled', 'type': 'bool', 'value': True},
                 {'title': 'Label:', 'name': 'label', 'type': 'str', 'value': 'Position 1'},
                 {'title': 'Position:', 'name': 'position', 'type': 'float', 'value': 0.0},
                 {'title': 'Set Current', 'name': 'set_current', 'type': 'action'},
             ]},
             {'title': 'Preset 2', 'name': 'preset2', 'type': 'group', 'children': [
                 {'title': 'Enabled:', 'name': 'enabled', 'type': 'bool', 'value': True},
                 {'title': 'Label:', 'name': 'label', 'type': 'str', 'value': 'Position 2'},
                 {'title': 'Position:', 'name': 'position', 'type': 'float', 'value': 1000.0},
                 {'title': 'Set Current', 'name': 'set_current', 'type': 'action'},
             ]},
             {'title': 'Preset 3', 'name': 'preset3', 'type': 'group', 'children': [
                 {'title': 'Enabled:', 'name': 'enabled', 'type': 'bool', 'value': True},
                 {'title': 'Label:', 'name': 'label', 'type': 'str', 'value': 'Position 3'},
                 {'title': 'Position:', 'name': 'position', 'type': 'float', 'value': 2000.0},
                 {'title': 'Set Current', 'name': 'set_current', 'type': 'action'},
             ]},
             {'title': 'Preset 4', 'name': 'preset4', 'type': 'group', 'children': [
                 {'title': 'Enabled:', 'name': 'enabled', 'type': 'bool', 'value': True},
                 {'title': 'Label:', 'name': 'label', 'type': 'str', 'value': 'Position 4'},
                 {'title': 'Position:', 'name': 'position', 'type': 'float', 'value': 3000.0},
                 {'title': 'Set Current', 'name': 'set_current', 'type': 'action'},
             ]},
         ]},
    ]

    def __init__(self, parent: gutils.DockArea, dashboard):
        super().__init__(parent, dashboard)
        self.actuator_module = None
        self.setup_ui()

        # Connect the "Set Current" actions
        for i in range(1, 5):
            self.settings.child('presets', f'preset{i}', 'set_current').sigActivated.connect(
                lambda checked, idx=i: self.set_preset_to_current(idx))
        
        # Connect the "Get Current Position" action
        self.settings.child('actuator_settings', 'get_current_pos').sigActivated.connect(
            self.update_current_position)

    def setup_docks(self):
        """Setup the docks layout"""
        # Create main buttons dock
        self.docks['buttons'] = gutils.Dock('Preset Positions')
        self.dockarea.addDock(self.docks['buttons'])
        
        # Create a widget to hold the preset buttons
        self.buttons_widget = QtWidgets.QWidget()
        self.buttons_layout = QtWidgets.QGridLayout()
        self.buttons_widget.setLayout(self.buttons_layout)
        self.docks['buttons'].addWidget(self.buttons_widget)
        
        # Create the large preset buttons
        self.preset_buttons = {}
        for i in range(1, 5):
            btn = QtWidgets.QPushButton()
            btn.setMinimumHeight(100)
            btn.setMinimumWidth(200)
            btn.setStyleSheet("""
                QPushButton {
                    font-size: 16pt;
                    font-weight: bold;
                    border: 2px solid #555;
                    border-radius: 10px;
                    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                                      stop:0 #4a90e2, stop:1 #357abd);
                    color: white;
                    padding: 10px;
                }
                QPushButton:hover {
                    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                                      stop:0 #5aa0f2, stop:1 #458acf);
                }
                QPushButton:pressed {
                    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                                      stop:0 #357abd, stop:1 #2a6a9f);
                }
                QPushButton:disabled {
                    background-color: #cccccc;
                    color: #666666;
                    border: 2px solid #999;
                }
            """)
            btn.clicked.connect(lambda checked, idx=i: self.goto_preset(idx))
            self.preset_buttons[i] = btn
            
            # Arrange buttons in 2x2 grid
            row = (i - 1) // 2
            col = (i - 1) % 2
            self.buttons_layout.addWidget(btn, row, col)
        
        # Add current position display below buttons
        self.position_display = QtWidgets.QLabel("Current Position: ---")
        self.position_display.setAlignment(QtCore.Qt.AlignCenter)
        self.position_display.setStyleSheet("""
            QLabel {
                font-size: 14pt;
                font-weight: bold;
                padding: 10px;
                background-color: #000000;
                border: 2px solid #555;
                border-radius: 5px;
                margin-top: 10px;
            }
        """)
        self.buttons_layout.addWidget(self.position_display, 2, 0, 1, 2)
        
        # Add emergency stop button
        self.stop_btn = QtWidgets.QPushButton("EMERGENCY STOP")
        self.stop_btn.setMinimumHeight(60)
        self.stop_btn.setStyleSheet("""
            QPushButton {
                font-size: 14pt;
                font-weight: bold;
                border: 3px solid #8B0000;
                border-radius: 10px;
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                                  stop:0 #ff4444, stop:1 #cc0000);
                color: white;
                padding: 10px;
                margin-top: 10px;
            }
            QPushButton:hover {
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                                  stop:0 #ff6666, stop:1 #dd2222);
            }
            QPushButton:pressed {
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                                  stop:0 #cc0000, stop:1 #990000);
            }
        """)
        self.stop_btn.clicked.connect(self.stop_motion)
        self.buttons_layout.addWidget(self.stop_btn, 3, 0, 1, 2)
        
        # Settings dock on the right
        self.docks['settings'] = gutils.Dock('Settings')
        self.dockarea.addDock(self.docks['settings'], 'right', self.docks['buttons'])
        self.docks['settings'].addWidget(self.settings_tree)
        
        # Create a dock for status/info at the bottom
        self.docks['status'] = gutils.Dock('Status Log')
        self.dockarea.addDock(self.docks['status'], 'bottom', self.docks['buttons'])
        
        # Add a text widget for status messages
        self.status_text = QtWidgets.QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setMaximumHeight(150)
        self.docks['status'].addWidget(self.status_text)
        
        # Update button states
        self.update_button_states()
        
        self.log_message("Extension initialized. Please select a Standa actuator module.")

    def setup_actions(self):
        """Create toolbar actions"""
        self.add_action('quit', 'Quit', 'close2', "Quit extension")
        self.add_action('refresh_actuator', 'Refresh Actuator', 'update2', 
                       "Refresh connection to actuator")
        self.add_action('update_position', 'Update Position', 'run2',
                       "Update current position display")
        self.add_action('save_presets', 'Save Presets', 'save2',
                        "Save presets to JSON")
        self.add_action('load_presets', 'Load Presets', 'load2',
                        "Load presets from JSON")        

    def connect_things(self):
        """Connect actions and signals to methods"""
        self.connect_action('quit', self.quit_fun)
        self.connect_action('refresh_actuator', self.refresh_actuator)
        self.connect_action('update_position', self.update_current_position)
        self.connect_action('save_presets', self.save_presets_to_json)
        self.connect_action('load_presets', self.load_presets_from_json)        

    def _collect_presets(self) -> dict:
        presets = {}
        for i in range(1, 5):
            path = ('presets', f'preset{i}')
            presets[f'preset{i}'] = {
                'enabled': self.settings[path + ('enabled',)],
                'label': self.settings[path + ('label',)],
                'position': self.settings[path + ('position',)],
            }
        return {'presets': presets}
    
    def _apply_presets(self, data: dict):
        presets = data.get('presets', {})
        for i in range(1, 5):
            key = f'preset{i}'
            if key not in presets:
                continue

            preset = presets[key]
            path = ('presets', key)

            self.settings.child(*path, 'enabled').setValue(
                preset.get('enabled', True))
            self.settings.child(*path, 'label').setValue(
                preset.get('label', f'Preset {i}'))
            self.settings.child(*path, 'position').setValue(
                preset.get('position', 0.0))

        self.update_button_states()

    def save_presets_to_json(self):
        start_dir = self.get_last_directory()

        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self.mainwindow,
            "Save Preset File",
            start_dir,
            "JSON Files (*.json);;All Files (*)"
        )

        if not filename:
            return

        if not filename.lower().endswith('.json'):
            filename += '.json'

        self.set_last_directory(filename)

        with open(filename, 'w') as f:
            json.dump(self._collect_presets(), f, indent=4)

        self.log_message(f"Presets saved to:\n{filename}")

    def load_presets_from_json(self):
        start_dir = self.get_last_directory()

        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self.mainwindow,
            "Load Preset File",
            start_dir,
            "JSON Files (*.json);;All Files (*)"
        )

        if not filename:
            return

        self.set_last_directory(filename)

        with open(filename, 'r') as f:
            data = json.load(f)

        self._apply_presets(data)
        self.log_message(f"Presets loaded from:\n{filename}")

    def value_changed(self, param):
        """Handle parameter value changes"""
        if param.name() == 'actuator_name':
            self.refresh_actuator()
        elif param.name() == 'enabled':
            # Update button states when preset is enabled/disabled
            self.update_button_states()
        elif param.name() == 'label':
            # Update button labels when preset label changes
            self.update_button_states()
        elif param.name() == 'position':
            # Update button labels when preset label changes
            self.update_button_states()            

    def refresh_actuator(self):
        """Refresh the connection to the actuator module"""
        actuator_name = self.settings['actuator_settings', 'actuator_name']
        try:
            self.actuator_module = self.dashboard.modules_manager.get_mod_from_name(
                actuator_name, 'act')
            if self.actuator_module is not None:
                self.log_message(f"Connected to actuator: {actuator_name}")
                self.actuator_module.current_value_signal.connect(lambda cur_pos: self.update_current_position(cur_pos))
                self.update_current_position()
            else:
                self.log_message(f"Warning: Could not find actuator '{actuator_name}'", 
                               level='warning')
        except Exception as e:
            self.log_message(f"Error connecting to actuator: {str(e)}", level='error')
            logger.exception(str(e))

    def update_current_position(self, current_pos=None):
        """Update the current position display"""
        if self.actuator_module is None:
            self.refresh_actuator()
        
        if self.actuator_module is not None:
            if current_pos is None:
                try:
                    current_pos = self.actuator_module._current_value
                    # Handle DataActuator object
                    if hasattr(current_pos, 'value'):
                        pos_value = current_pos.value()
                    else:
                        pos_value = float(current_pos)
                    
                    self.settings.child('actuator_settings', 'current_position').setValue(pos_value)
                    self.position_display.setText(f"Current Position: {pos_value:.2f}")
                    return pos_value
                except Exception as e:
                    self.log_message(f"Error getting current position: {str(e)}", level='error')
                    self.position_display.setText("Current Position: ERROR")
                    return None
            else:
                # Handle DataActuator object
                if hasattr(current_pos, 'value'):
                    pos_value = current_pos.value()
                else:
                    pos_value = float(current_pos)
                
                self.settings.child('actuator_settings', 'current_position').setValue(pos_value)
                self.position_display.setText(f"Current Position: {pos_value:.2f}")
        else:
            self.position_display.setText("Current Position: Not Connected")
        return None

    def set_preset_to_current(self, preset_num: int):
        """Set a preset position to the current motor position"""
        current_pos = self.update_current_position()
        if current_pos is not None:
            self.settings.child('presets', f'preset{preset_num}', 'position').setValue(
                current_pos)
            label = self.settings['presets', f'preset{preset_num}', 'label']
            self.log_message(f"Set '{label}' to position {current_pos}")

    def goto_preset(self, preset_num: int):
        """Move the actuator to a preset position"""
        if self.actuator_module is None:
            self.log_message("No actuator connected. Please set actuator name and refresh.", 
                           level='warning')
            return
        
        preset_path = ('presets', f'preset{preset_num}')
        
        # Check if preset is enabled
        if not self.settings[preset_path + ('enabled',)]:
            self.log_message(f"Preset {preset_num} is disabled", level='warning')
            return
        
        target_position = self.settings[preset_path + ('position',)]
        label = self.settings[preset_path + ('label',)]
        
        try:
            self.log_message(f"Moving to '{label}' at position {target_position}")
            
            # Move the actuator
            from pymodaq.control_modules.move_utility_classes import DataActuator
            self.actuator_module.move_abs(DataActuator(data=target_position))
            
            self.log_message(f"Move command sent to '{label}'")
            
        except Exception as e:
            self.log_message(f"Error moving to preset: {str(e)}", level='error')
            logger.exception(str(e))

    def stop_motion(self):
        """Emergency stop the actuator"""
        if self.actuator_module is not None:
            try:
                self.actuator_module.stop_motion()
                self.log_message("EMERGENCY STOP activated", level='warning')
            except Exception as e:
                self.log_message(f"Error stopping motion: {str(e)}", level='error')
        else:
            self.log_message("No actuator connected", level='warning')

    def update_button_states(self):
        """Update button labels and enabled states based on preset settings"""
        for i in range(1, 5):
            preset_path = ('presets', f'preset{i}')
            enabled = self.settings[preset_path + ('enabled',)]
            label = self.settings[preset_path + ('label',)]
            position = self.settings[preset_path + ('position',)]
            
            # Update the large button
            btn = self.preset_buttons[i]
            btn_text = f"{label}\n{position:.2f}"
            btn.setText(btn_text)
            btn.setEnabled(enabled)

    def log_message(self, message: str, level: str = 'info'):
        """Log a message to the status text widget"""
        import datetime
        timestamp = datetime.datetime.now().strftime('%H:%M:%S')
        
        if level == 'error':
            color = 'red'
            prefix = 'ERROR'
        elif level == 'warning':
            color = 'orange'
            prefix = 'WARNING'
        else:
            color = 'black'
            prefix = 'INFO'
        
        formatted_msg = f'<span style="color:{color}">[{timestamp}] {prefix}: {message}</span>'
        self.status_text.append(formatted_msg)
        
        # Also log to logger
        if level == 'error':
            logger.error(message)
        elif level == 'warning':
            logger.warning(message)
        else:
            logger.info(message)

    def get_last_directory(self, default=None):
        try:
            last_dir = standa_config[('last_directory',)]
            if last_dir and Path(last_dir).exists():
                return last_dir
        except Exception:
            pass

        return default or str(Path.home())

    def set_last_directory(self, filepath: str):
        directory = str(Path(filepath).parent)

        standa_config[('last_directory',)] = directory
        standa_config.save()

    def quit_fun(self):
        """Close the extension"""
        self.mainwindow.close()

def main():
    from pymodaq_gui.utils.utils import mkQApp
    from pymodaq.utils.gui_utils.loader_utils import load_dashboard_with_preset

    app = mkQApp('StandaPresets')

    preset_file_name = config_pymodaq('presets', 'default_preset_for_standa')
    dashboard, extension, win = load_dashboard_with_preset(preset_file_name, EXTENSION_NAME)
    app.exec()

    return dashboard, extension, win


if __name__ == '__main__':
    main()