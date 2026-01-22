from qtpy import QtWidgets, QtCore, QtGui
import numpy as np
import os
import xml.etree.ElementTree as ET
import tomllib
from pathlib import Path
import pyqtgraph as pg
from typing import Any
from time import perf_counter

from pymodaq.utils import gui_utils as gutils
from pymodaq.utils.config import Config
from pymodaq.utils.logger import set_logger, get_module_name
from pymodaq.utils.data import DataToExport, DataFromPlugins, Axis
from pymodaq.utils.managers.modules_manager import ModulesManager

from pymodaq_gui.plotting.data_viewers.viewer2D import Viewer2D
from pymodaq.control_modules.daq_viewer import DAQ_Viewer
from pymodaq.control_modules.daq_move import DAQ_Move
from pymodaq.control_modules.daq_move_ui.factory import ActuatorUIFactory

from pymodaq_gui.parameter import utils as putils
from pymodaq.utils.gui_utils.widgets.lcd import LCD
from pymodaq_gui.plotting.items.roi import EllipseROI
from pyqtgraph.parametertree import Parameter, ParameterTree
import random

from pymodaq_plugins_standa.utils import Config as PluginConfig
from scipy import ndimage

logger = set_logger(get_module_name(__file__))

main_config = Config()
plugin_config = PluginConfig()


class BeamStabilizer(gutils.CustomApp):
    worker_result_signal = QtCore.Signal(object)
    get_crosshair_data_signal = QtCore.Signal(object, object)
    params = [
        {'title': 'Gaussian overlay', 'name': 'roi', 'type': 'group', 'children': [
            {'title': 'Show ellipse', 'name': 'show_ellipse', 'type': 'led_push', 'value': False, 'default': False},
            {'title': 'Show lineout', 'name': 'show_lineout', 'type': 'led_push', 'value': False, 'default': False},
            {'title': 'Pixel calibration', 'name': 'pixel_calibration', 
            'type': 'float', 'value': 1.0, 'default': 1.0, 'tip': 'Distance per pixel'},
            {'title': 'Units', 'name': 'units', 'type': 'list', 'limits': ['pixels', 'um', 'mm', 'cm'], 'default': 'pixels'},
            {'title': 'Save current ROI', 'name': 'save_ellipse', 'type': 'bool_push', 'value': False, 'default': False},
        ]},
        {'title': 'Load saved ROIs', 'name': 'load_saved_rois', 'type': 'group', 'children': [
            {'title': 'Saving Base Path:', 'name': 'load_rois_file', 'type': 'browsepath', 'value': '', 'filetype': True},
            {'title': 'Load ROIs', 'name': 'load_rois', 'type': 'bool_push', 'value': False, 'default': False}
        ]},
        {'title': 'Config Base Path:', 'name': 'config_base_path', 'type': 'browsepath', 'value': '', 'filetype': False},
    ]

    def __init__(self, parent: gutils.DockArea, config_name: str):
        super().__init__(parent)

        self.pixel_calibration = 1.0
        self.units = 'pixels'

        self.viewer = {
            'viewer': None,
            'roi': None,
            'lcd': None,
            'show_ellipse': False,
            'show_lineout': False,
            'worker': None,
            'thread': None,
            'input': None,
            'latest_frame': None,
            'worker_busy': False,
            '_saving_roi': False,
            '_loading_roi': False,
            'frame_id': 0,
        }
        self.flip_ud_state = False
        self.flip_lr_state = False
        self._background = np.array([])
        self.pid_module = None
        self.pid_window = None
        self.modules_manager = None
        self.scan_module = None
        self.config_name = config_name
        self.config = {}
        self.qsettings = QtCore.QSettings("PyMoDAQ", "BeamStabilizer")
        stored = self.qsettings.value('beamstabilizer_configs/basepath', type=str)
        if stored:
            self.configs_dir = Path(str(stored))
        else:
            self.configs_dir = Path.home() / "Documents"

        self.setup_config()
        self.setup_ui()
        self._setup_worker_threads()

    def setup_docks(self):
        self.docks['viewer2D'] = gutils.Dock('Beam Tracker')
        self.dockarea.addDock(self.docks['viewer2D'])
        self.target_viewer = Viewer2D(QtWidgets.QWidget())
        self.docks['viewer2D'].addWidget(self.target_viewer.parent)

        self.docks['lcds'] = gutils.Dock('Beam properties')
        self.dockarea.addDock(self.docks['lcds'], 'right', self.docks['viewer2D'])
        self.lcd = LCD(QtWidgets.QWidget(), Nvals=5, digits=4, labels=['x0', 'y0', f'd_major ({self.units})',
                                                            f'd_minor ({self.units})', 'phi (deg.)'])
        self.docks['lcds'].addWidget(self.lcd.parent)

        self.docks['settings'] = gutils.Dock('Settings')
        self.dockarea.addDock(self.docks['settings'], 'bottom')
        self.docks['settings'].addWidget(self.settings_tree)
       
        dashboard = QtWidgets.QMainWindow()
        dashboard_dockarea = gutils.DockArea()
        dashboard.setCentralWidget(dashboard_dockarea)
        if self.config['detector'] != "Mock":
            title = self.config['camera_name']
        else:
            title = "Mock"
        self.detector_modules = [DAQ_Viewer(dashboard_dockarea, title=title, daq_type='DAQ2D')]
        self.detector_modules[0].detector = self.config['detector']
        if self.config['detector'] != "Mock":
            self.detector_modules[0].settings.child('detector_settings', 'camera_list').setValue(self.config['camera_name'])
        self.detector_modules[0].init_signal.connect(lambda _: self._on_viewer_initialized(detector_modules=self.detector_modules[0], target_viewer=self.target_viewer))
        self.detector_modules[0].init_hardware_ui(do_init=self.config['init_cam'])

        self.docks['actuator'] = gutils.Dock("Actuator")
        self.dockarea.addDock(self.docks['actuator'])
        self.actuators_modules = [DAQ_Move(QtWidgets.QWidget(), title='Target Vertical Director', ui_identifier='Original')]
        self.docks['actuator'].addWidget(self.actuators_modules[0].parent)
        self.actuators_modules[0].actuator = self.config['actuator']
        self.actuators_modules[0].settings.child('move_settings', 'actor_name').setValue(self.config['stage_actor_name'])
        self.actuators_modules[0].settings.child('move_settings', 'host').setValue(self.config['stage_host_ip'])
        self.actuators_modules[0].init_signal.connect(lambda _: self._on_move_intialized(stage_move=self.actuators_modules[0]))
        self.actuators_modules[0].init_hardware_ui(do_init=self.config['init_stage'])

    def setup_actions(self):
        self.add_action('grab', 'Grab', 'camera', checkable=True, tip="Grab from camera") 
        self.add_action('stop', 'Stop', 'stop', tip="Stop grabbing from camera") 
        self.add_action('quit', 'Quit', 'close2', "Quit program")
        self.add_action('show', 'Show/hide', 'read2', "Show Hide DAQ modules", checkable=True)
        self.add_action("do_pid", "PID module")
        self.add_action("take_background", "Take Background")

    def connect_things(self):
        # Connect app functionalities
        self.connect_action('grab', self.detector_modules[0].grab)
        self.connect_action('stop', self.detector_modules[0].stop)
        self.connect_action('show', lambda do_show: self.show_devices(do_show))
        self.connect_action('quit', self.quit_app)
        self.connect_action('do_pid', self.load_pid_module)
        self.connect_action('take_background', self.take_background)

        # Connect data handling
        self.detector_modules[0].grab_done_signal.connect(
            lambda dte: self._on_new_frame(dte)
        )
        self.worker_result_signal.connect(lambda fit: self._update_fit_roi(fit))
        self.get_crosshair_data_signal.connect(lambda roi_bounds, fit: self.get_crosshair_data_within_roi(roi_bounds, fit))

        # Connect flip up/lr signals
        self.target_viewer.view.connect_action('flip_ud', self._on_flip_ud_clicked)
        self.target_viewer.view.connect_action('flip_lr', self._on_flip_lr_clicked)

    def set_camera_presets(self, detector_modules, target_viewer):
        if self.config['detector'] != "Mock":
            cam_props = self.config.get("camera_properties", {})
            for key, val in cam_props.items():
                if isinstance(val, dict):
                    for prop, value in val.items():
                        param = detector_modules.settings.child('detector_settings', str(key), str(prop))
                        if param is not None:
                            param.setValue(value)
                            param.sigValueChanged.emit(param, param.value())
            if self.config['references']['use_references']:
                xml_path = f"{self.configs_dir}/{self.config['references']['reference']}.xml"
                self.load_roi_from_xml(target_viewer, xml_path)
            param = self.settings.param('config_base_path')
            param.blockSignals(True)
            param.setValue(self.configs_dir)
            param.blockSignals(False)

    def set_stage_presets(self, stage_move):
        stage_props = self.config.get("stage_properties", {})
        for key, val in stage_props.items():
            if isinstance(val, dict):
                for prop, value in val.items():
                    param = stage_move.settings.child('move_settings', str(key), str(prop))
                    if param is not None:
                        param.setValue(value)
                        param.sigValueChanged.emit(param, param.value())

    def setup_config(self):
        config_template_path = Path(__file__).parent.joinpath(f'{self.configs_dir}/{self.config_name}.toml')        
        with open(config_template_path, "rb") as f:
            self.config = tomllib.load(f)


    def _setup_worker_threads(self):
            th = QtCore.QThread(self)
            w = BeamSizeWorker()
            w.moveToThread(th)
            th.start()

            class WorkerInput(QtCore.QObject):
                trigger = QtCore.Signal(object, int)

            inp = WorkerInput()
            inp.trigger.connect(w.process, QtCore.Qt.QueuedConnection)

            w.result_ready.connect(lambda values, fid: self._on_worker_result(values, fid))
            w.error.connect(lambda err_msg, fid: self._on_worker_error(err_msg, fid))

            self.viewer = {
                'viewer': self.target_viewer,
                'roi': None,
                'lcd': self.lcd,
                'show_ellipse': False,
                'show_lineout': False,
                'worker': w,
                'thread': th,
                'input': inp,
                'latest_frame': None,
                'worker_busy': False,
                '_saving_roi': False,
                '_loading_roi': False,
                'frame_id': 0,
            }

    def _on_new_frame(self, dte):
        self.show_data(dte)
        self._kick_analysis()            

    def _kick_analysis(self):
        viewer_info = self.viewer
        viewer = viewer_info['viewer']
        viewer_info['frame_id'] += 1
        fid = viewer_info['frame_id']
        if viewer_info['latest_frame'] is None or viewer_info['worker_busy']:
            return

        frame = viewer_info['latest_frame']
        # Flip data if necessary
        if self.flip_ud_state:
            frame = np.flipud(frame)
        if self.flip_lr_state:
            frame = np.fliplr(frame)

        # Create copy of data for processing
        try:
            frame_to_process = np.asarray(frame, dtype=frame.dtype).copy(order='C')
        except Exception:
            frame_to_process = frame.copy()

        nrows, ncols = frame_to_process.shape[:2]

        # Crop data to user-defined ROI if necessary
        if viewer.view.ROIselect.isVisible():
            x0, y0 = viewer.view.ROIselect.pos()
            width, height = viewer.view.ROIselect.size()
            y0, x0 = int(y0), int(x0)
            y1, x1 = min(y0 + int(height), nrows), min(x0 + int(width), ncols)
        else:
            x0, y0 = 0, 0
            y1, x1 = nrows, ncols

        frame_to_process = frame_to_process[y0:y1, x0:x1]

        # Initialize background if not set or shape mismatch
        if (self._background.size == 0) or (self._background.shape != frame_to_process.shape):
            self._background = np.zeros_like(frame_to_process, dtype=frame_to_process.dtype)

        # Subtract background
        frame_to_process = frame_to_process.astype(float) - self._background.astype(float)

        # Isolate main spot @ 1/e^2
        frame_to_process = isolate_main_spot(frame_to_process, threshold_ratio=0.135)

        viewer_info['worker_busy'] = True
        viewer_info['input'].trigger.emit(frame_to_process, fid) # Forward data for fitting

    @QtCore.Slot(object, int)
    def _on_worker_result(self, fit, frame_id):
        info = self.viewer
        try:
            # Extract parameters from fit and calibrate if necessary
            x, y, dmajor, dminor, theta = fit
            dmajor_cal = dmajor * self.pixel_calibration
            dminor_cal = dminor * self.pixel_calibration
            roi_pos = info['viewer'].view.ROIselect.pos()
            
            try:
                # Forward fit parameters to PID loop if necessary
                if self.pid_module is not None:
                    rt = getattr(self.pid_module, 'runner_thread', None)
                    if rt is not None and hasattr(rt, 'pid_runner'):
                        rt.pid_runner.update_gaussian_fit.emit(fit, roi_pos, 0)
            except Exception as e:
                logger.exception("Failed to forward gaussian fit to PID runner: ", e)

            # Display current fit parameters in GUI
            info['lcd'].setvalues([np.array([t]) for t in (x+roi_pos.x(), y+roi_pos.y(), dmajor_cal, dminor_cal, rad2deg(theta))])
            self.worker_result_signal.emit(fit) # Forward fit for ellipse drawing
        except Exception as e:
            logger.exception(f"_on_worker_result failed: ", e)
        finally:
            info['worker_busy'] = False


    @QtCore.Slot(str, int)
    def _on_worker_error(self, msg, frame_id):
        logger.error(msg)
        info = self.viewer
        if frame_id != info['frame_id']:
            return
        info['worker_busy'] = False

    def _update_fit_roi(self, fit):
        viewer_info = self.viewer
        viewer = viewer_info['viewer']

        x, y, dmajor, dminor, theta = fit

        if (not viewer_info['show_ellipse'] or viewer_info['latest_frame'] is None
            or dmajor == 0 or dminor == 0):
            try:
                if self.viewer['roi'] is not None:
                    viewer.view.plotitem.removeItem(self.viewer['roi'])
                    self.viewer['roi'] = None
            except Exception:
                pass
            return

        if viewer.view.ROIselect.isVisible():
            roi_x0, roi_y0 = viewer.view.ROIselect.pos()
            roi_w, roi_h = viewer.view.ROIselect.size()
        else:
            roi_x0, roi_y0 = 0, 0
            roi_h, roi_w = viewer_info['latest_frame'].shape

        global_x = roi_x0 + x
        global_y = roi_y0 + y
        global_center = QtCore.QPointF(global_x, global_y)

        xmin, xmax, ymin, ymax = self.get_bounding_rect(dmajor/2, dminor/2, theta, center=(global_x, global_y))
        width, height = np.abs(xmax-xmin), np.abs(ymax-ymin)
        size_view = [width, height]

        # remove previous ROI if present
        if self.viewer.get('roi') is not None:
            try:
                viewer.view.plotitem.removeItem(self.viewer['roi'])
            except Exception:
                pass
        roi = EllipseROI(index=0, pos = [global_x - width/2, global_y - height/2], 
                         size = size_view)
        roi.set_center(pg.Point((global_x, global_y)))
        roi.setAngle(theta, center=(0.5, 0.5))
        roi.setPen(QtGui.QPen(QtGui.QColor(255,0,0),0.05))
        roi.setZValue(10) 
        roi.setAcceptedMouseButtons(QtCore.Qt.NoButton)
        self.viewer['roi'] = roi
        viewer.view.plotitem.addItem(roi)
        viewer.view.set_crosshair_position(global_center.x(), global_center.y())
        roi.setVisible(True)

        if viewer_info['show_lineout']:
            roi_bounds = (roi_x0, roi_y0, roi_w, roi_h)
            gaussian_fit = (x, y, dmajor/2, dminor/2, theta)
            self.get_crosshair_data_signal.emit(roi_bounds, gaussian_fit)

    def get_crosshair_data_within_roi(self, roi_bounds, gaussian_fit):
        viewer_info = self.viewer
        viewer = viewer_info['viewer']
        frame = viewer_info['latest_frame']
        # Flip data if necessary
        if self.flip_ud_state:
            frame = np.flipud(frame)
        if self.flip_lr_state:
            frame = np.fliplr(frame)        
        roi_x0, roi_y0, roi_w, roi_h = roi_bounds
        gaussian_x, gaussian_y, gaussian_wx, gaussian_wy, gaussian_theta = gaussian_fit

        centroid = viewer.view.get_crosshair_position()
        cx, cy = int(centroid[0]), int(centroid[1])

        x0, x1 = int(max(0, min(frame.shape[1], roi_x0))), int(max(0, min(frame.shape[1], roi_x0 + roi_w)))
        y0, y1 = int(max(0, min(frame.shape[0], roi_y0))), int(max(0, min(frame.shape[0], roi_y0 + roi_h)))

        x_axis = np.linspace(1, int(roi_w), int(roi_w))
        y_axis = np.linspace(1, int(roi_h), int(roi_h))

        # Horizontal lineout
        if x0 <= cx < x1:
            hor_raw = np.squeeze(frame[cy, x0:x1])
            gauss_hor = gaussian_lineout_x(x_axis, gaussian_x, gaussian_y, gaussian_wx, gaussian_wy, gaussian_theta, cy - roi_y0)
            min_len = min(len(hor_raw), len(gauss_hor))
            hor_raw, gauss_hor = hor_raw[:min_len], gauss_hor[:min_len]
            if np.max(gauss_hor) != 0:
                gauss_hor = gauss_hor * (np.max(hor_raw)/np.max(gauss_hor))
            x_axis_trim = np.linspace(1, len(hor_raw), len(hor_raw))
            dwa_hor = DataFromPlugins(name='hor', data=[hor_raw, gauss_hor], dim='Data1D', labels=['crosshair_hor','gaussian_hor'], axes=[Axis(label='Pixels', data=x_axis_trim)])
            viewer.view.lineout_viewers['hor'].view.display_data(dwa_hor, displayer='crosshair')

        # Vertical lineout
        if y0 <= cy < y1:
            ver_raw = np.squeeze(frame[y0:y1, cx])
            gauss_ver = gaussian_lineout_y(y_axis, gaussian_x, gaussian_y, gaussian_wx, gaussian_wy, gaussian_theta, cx - roi_x0)
            min_len = min(len(ver_raw), len(gauss_ver))
            ver_raw, gauss_ver = ver_raw[:min_len], gauss_ver[:min_len]
            if np.max(gauss_ver) != 0:
                gauss_ver = gauss_ver * (np.max(ver_raw)/np.max(gauss_ver))
            y_axis_trim = np.linspace(1, len(ver_raw), len(ver_raw))
            dwa_ver = DataFromPlugins(name='ver', data=[ver_raw, gauss_ver], dim='Data1D', labels=['crosshair_ver','gaussian_ver'], axes=[Axis(label='Pixels', data=y_axis_trim)])
            viewer.view.lineout_viewers['ver'].view.display_data(dwa_ver, displayer='crosshair')

    def show_data(self, dte: DataToExport):
        info = self.viewer
        try:
            dsrc = dte.get_data_from_source('raw')
            data2d_list = dsrc.get_data_from_dim('Data2D')
            data2d = data2d_list[0]

            arr = data2d[0] if isinstance(data2d, (list, tuple)) else data2d
            arr = np.asarray(arr)
            if arr.ndim == 3 and arr.shape[0] == 1:
                arr = arr[0]
            if not arr.flags['C_CONTIGUOUS']:
                arr = np.ascontiguousarray(arr)

            info['viewer'].show_data(data2d)
            info['latest_frame'] = arr
        except Exception as e:
            logger.error(f"show_data failed: {e}")

    def take_background(self):
        if self.viewer['latest_frame'] is None:
            logger.warning("No frame available to save as background.")
            return

        frame = self.viewer['latest_frame']
        nrows, ncols = frame.shape[:2]

        if self.viewer['viewer'].view.ROIselect.isVisible():
            x0, y0 = self.viewer['viewer'].view.ROIselect.pos()
            width, height = self.viewer['viewer'].view.ROIselect.size()
            y0, x0 = int(y0), int(x0)
            y1, x1 = min(y0 + int(height), nrows), min(x0 + int(width), ncols)
        else:
            x0, y0 = 0, 0
            y1, x1 = nrows, ncols

        self._background = frame[y0:y1, x0:x1].astype(float)
        logger.info(f"Background saved (shape={self._background.shape}).")
        print(f"Background saved (shape={self._background.shape}).")

    def value_changed(self, param):
        viewer_info = self.viewer
        viewer = viewer_info['viewer']
        fit_roi = viewer_info['roi']
        lcd = viewer_info['lcd']

        if param.name() == 'show_ellipse':
            viewer_info['show_ellipse'] = param.value()
            viewer.view.show_hide_crosshair(show=param.value())

        elif param.name() == 'show_lineout':
            viewer_info['show_lineout'] = param.value()
            if param.value():                
                viewer.view.prepare_image_widget_for_lineouts()
                viewer.view.lineout_viewers['hor'].view.add_data_displayer('crosshair_hor', 'red')
                viewer.view.lineout_viewers['ver'].view.add_data_displayer('crosshair_ver', 'red')
            else:
                viewer.view.prepare_image_widget_for_lineouts(1)
                viewer.view.lineout_viewers['hor'].view.remove_data_displayer('crosshair_hor')
                viewer.view.lineout_viewers['ver'].view.remove_data_displayer('crosshair_ver')

        elif param.name() == 'pixel_calibration':
            self.pixel_calibration = param.value()

        elif param.name() == 'units':
            self.units = param.value()
            new_labels = ['x0', 'y0', f'd_major ({self.units})', f'd_minor ({self.units})', 'phi (deg.)']
            qlabels = lcd.parent.findChildren(QtWidgets.QLabel)
            for lbl, new_text in zip(qlabels, new_labels):
                lbl.setText(new_text)

        elif param.name() == 'save_ellipse':
            if viewer_info['_saving_roi'] or fit_roi is None:
                return
            viewer_info['_saving_roi'] = True
            try:
                viewer.view.roi_manager.add_roi_programmatically('EllipseROI')
                roi_dict = viewer.view.roi_manager.ROIs
                latest_key, latest_roi = list(roi_dict.items())[-1]
                pos = fit_roi.pos()
                size = fit_roi.size()
                angle = fit_roi.angle()
                latest_roi.setPos(pos)
                latest_roi.setSize(size)
                latest_roi.setAngle(angle)
                r, g, b = [random.randint(0, 255) for _ in range(3)]
                latest_roi.setPen(QtGui.QPen(QtGui.QColor(r, g, b), 0.1))
            finally:
                viewer_info['_saving_roi'] = False
                param.blockSignals(True)
                param.setValue(False)
                param.blockSignals(False)                

        elif param.name() == 'load_rois':
            xml_path = self.settings.child('load_saved_rois', 'load_rois_file').value()
            if not xml_path or not os.path.isfile(xml_path) or viewer_info['_loading_roi']:
                return
            viewer_info['_loading_roi'] = True
            try:
                self.load_roi_from_xml(viewer, xml_path)
            finally:
                viewer_info['_loading_roi'] = False
                param.blockSignals(True)
                param.setValue(False)
                param.blockSignals(False)

        elif param.name() == 'config_base_path':
            self.qsettings.setValue('beamstabilizer_configs/basepath', param.value())
            logger.info("Set the default config base path for the BeamStabilizer app to: ", self.qsettings.value('beamstabilizer_configs/basepath'))
            self.qsettings.sync()

    def load_roi_from_xml(self, viewer, xml_path):
        tree = ET.parse(xml_path)
        root = tree.getroot()

        for roi_group in root.findall("./*"):  
            roi_type = roi_group.find('roi_type').text

            pos_x = float(roi_group.find('./position/x').text)
            pos_y = float(roi_group.find('./position/y').text)
            width = float(roi_group.find('./size/width').text)
            height = float(roi_group.find('./size/height').text)
            angle = float(roi_group.find('./angle').text)

            color_node = roi_group.find('Color')
            color = None
            if color_node is not None:
                color = eval(color_node.text)

            viewer.view.roi_manager.add_roi_programmatically(roi_type)
            roi_dict = viewer.view.roi_manager.ROIs
            roi_key, roi = list(roi_dict.items())[-1]
            roi.setPos([pos_x, pos_y])
            roi.setSize([width, height])
            roi.setAngle(angle)
            roi.setAcceptedMouseButtons(QtCore.Qt.NoButton)
            if color:
                roi.setPen(color)                    

    def get_bounding_rect(self, a, b, theta, center=(0.0, 0.0)):
        cx, cy = center

        x_max = np.sqrt((a * np.cos(theta))**2 + (b * np.sin(theta))**2)
        y_max = np.sqrt((a * np.sin(theta))**2 + (b * np.cos(theta))**2)

        x_min, x_max = cx - x_max, cx + x_max
        y_min, y_max = cy - y_max, cy + y_max

        return x_min, x_max, y_min, y_max            

    def show_devices(self, do_show: bool):
        self.detector_modules[0].parent.parent().setVisible(do_show)

    def load_pid_module(self, win=None):
        if win is None or not win:
            self.pid_window = QtWidgets.QMainWindow()
        else:
            self.pid_window = win
        self.pid_window.setWindowFlags(
            QtCore.Qt.Window
            | QtCore.Qt.WindowTitleHint
            | QtCore.Qt.WindowMinimizeButtonHint
            | QtCore.Qt.WindowMaximizeButtonHint
        )
        dockarea = gutils.DockArea()
        self.pid_window.setCentralWidget(dockarea)
        self.pid_window.setWindowTitle("PID Controller")
        self.pid_module = DAQ_PID(dockarea=dockarea, dashboard=self)
        self.pid_window.show()
        return self.pid_module
    
    def add_move(
            self,
            plug_name: str = None,
            plug_settings: Parameter = None,
            plug_type: str = None,
            move_docks: list[gutils.Dock] = None,
            move_forms: list[QtWidgets.QWidget] = None,
            actuators_modules: list[DAQ_Move] = None,
            ui_identifier: str = None,
            **kwargs
    ) -> DAQ_Move:        
        if move_docks is None:
            move_docks = []
        if move_forms is None:
            move_forms = []
        if actuators_modules is None:
            actuators_modules = []      

        if ui_identifier is not None:
            pass
        elif plug_settings is None:
            ui_identifier = "Original"
        else:
            try:
                ui_identifier = plug_settings["main_settings", "ui_type"]
            except KeyError:
                ui_identifier = "Original"

        is_compact = (
            ActuatorUIFactory.get(ui_identifier).is_compact
            if ui_identifier is not None
            else False
        )

        if is_compact:
            if self.compact_actuator_dock is None:
                self.compact_actuator_dock = gutils.Dock("Simple Actuators")
                self.compact_actuator_dock.layout.setSpacing(0)
                self.compact_actuator_dock.layout.setContentsMargins(0, 0, 0, 0)

            dock = self.compact_actuator_dock
            self.docks['settings'].area.addDock(dock, "bottom")
        else:
            dock = gutils.Dock(plug_name, size=(150, 250))
            move_docks.append(dock)

            if len(move_docks) == 1:
                self.dockarea.addDock(dock, "right", self.docks['settings'])
            else:
                self.dockarea.addDock(dock, "above", move_docks[-2])

        move_forms.append(QtWidgets.QWidget())
        mov_mod_tmp = DAQ_Move(move_forms[-1], plug_name, ui_identifier=ui_identifier)

        mov_mod_tmp.actuator = plug_type
        QtWidgets.QApplication.processEvents()
        mov_mod_tmp.manage_ui_actions("quit", "setEnabled", False)

        if plug_settings is not None:
            try:
                putils.set_param_from_param(mov_mod_tmp.settings, plug_settings)
            except KeyError as e:
                mssg = (
                    f"Could not set this setting: {str(e)}\n"
                    f"The Preset is no more compatible with the plugin {plug_type}"
                )
                logger.warning(mssg)
                self.splash_sc.showMessage(mssg)
        QtWidgets.QApplication.processEvents()

        mov_mod_tmp.bounds_signal[bool].connect(self.do_stuff_from_out_bounds)
        dock.addWidget(move_forms[-1])

        actuators_modules.append(mov_mod_tmp)
        return mov_mod_tmp    
    
    def add_move_from_extension(
        self, name: str, instrument_name: str, instrument_controller: Any,
            ui_identifier = None,
            **kwargs
    ):
        """Specific method to add a DAQ_Move within the Dashboard. This Particular actuator
        should be defined in the plugin of the extension and is used to mimic an actuator while
        move_abs is actually triggering an action on the extension which loaded it

        For an exemple, see the PyMoDAQ builtin PID extension

        Parameters
        ----------
        name: str
            The name to print on the UI title
        instrument_name: str
            The name of the instrument class, for instance PID for the daq_move_PID
            module and the DAQ_Move_PID instrument class
        instrument_controller: object
            whatever object is used to communicate between the instrument module and the extension
            which created it
        ui_identifier: str
            One of the possible registered UI
        kwargs: named arguments to be passed to add_move
        """
        actuator = self.add_move(name, None, instrument_name, [], [], [],
                                 ui_identifier=ui_identifier,
                                 **kwargs)
        actuator.controller = instrument_controller
        actuator.master = False
        actuator.init_hardware_ui()
        QtWidgets.QApplication.processEvents()
        self.poll_init(actuator)
        QtWidgets.QApplication.processEvents()

        # Update actuators modules and module manager
        self.actuators_modules.append(actuator)
        self.update_module_manager()

    def do_stuff_from_out_bounds(self, out_of_bounds: bool):
        if out_of_bounds:
            logger.warning(f"Some actuators reached their bounds")
            if self.scan_module is not None:
                logger.warning(f"Stopping the DAQScan for out of bounds")
                self.scan_module.stop_scan()

    def poll_init(self, module):
        is_init = False
        tstart = perf_counter()
        while not is_init:
            QtCore.QThread.msleep(1000)
            QtWidgets.QApplication.processEvents()
            is_init = module.initialized_state
            if perf_counter() - tstart > 60:  # timeout of 60sec
                break
        return is_init
    
    def update_module_manager(self):
        if self.modules_manager is None:
            self.modules_manager = ModulesManager(
                self.detector_modules, self.actuators_modules, parent_name="BeamStabilizer"
            )
        else:
            self.modules_manager.actuators_all = self.actuators_modules
            self.modules_manager.detectors_all = self.detector_modules

    def _on_viewer_initialized(self, detector_modules, target_viewer):
        self.set_camera_presets(detector_modules, target_viewer)

    def _on_move_intialized(self, stage_move):
        self.set_stage_presets(stage_move)

    def _on_flip_ud_clicked(self):
        if self.target_viewer.view.is_action_checked('flip_ud'):
            self.flip_ud_state = True
        else:
            self.flip_ud_state = False

    def _on_flip_lr_clicked(self):            
        if self.target_viewer.view.is_action_checked('flip_lr'):
            self.flip_lr_state = True
        else:
            self.flip_lr_state = False

    def cleanup_threads(self):
        info = self.viewer
        th = info.get('thread')
        if th is not None:
            try:
                th.quit()
                th.wait()
            except Exception as e:
                logger.error(f"Error stopping worker thread: {e}")

    def quit_app(self):
        try:
            self.cleanup_threads()
        except Exception as e:
            logger.error(f"Error while quitting app: {e}")
        try:            
            self.actuators_modules[0].quit_fun()
        except Exception as e:
            logger.error(f"Error while quitting app: {e}")            
        try:
            if self.pid_window is not None:
                self.pid_window.close()
        except Exception as e:
            logger.error(f"Error while quitting app: {e}")
        try:            
            self.mainwindow.close()
        except Exception as e:
            logger.error(f"Error while quitting app: {e}")


def gaussian_lineout_x(x_axis, x0, y0, wx, wy, theta, y_cross, A=1.0):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    X = x_axis - x0
    Y = y_cross - y0
    x_rot = X * cos_t + Y * sin_t
    y_rot = -X * sin_t + Y * cos_t    
    exponent = (2*(x_rot / wx)**2) + (2*(y_rot / wy)**2)
    return A * np.exp(-exponent)

def gaussian_lineout_y(y_axis, x0, y0, wx, wy, theta, x_cross, A=1.0):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    X = x_cross - x0
    Y = y_axis - y0
    x_rot = X * cos_t + Y * sin_t
    y_rot = -X * sin_t + Y * cos_t    
    exponent = (2*(x_rot / wx)**2) + (2*(y_rot / wy)**2)
    return A * np.exp(-exponent)

def gaussian_2d(x, y, x0, y0, wx, wy, theta, A=1.0, offset=0.0):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    X = x - x0
    Y = y - y0
    x_rot = X * cos_t + Y * sin_t
    y_rot = -X * sin_t + Y * cos_t
    exponent = (2*(x_rot / wx)**2) + (2*(y_rot / wy)**2)
    return A * np.exp(-exponent) + offset

def compute_gaussianness(data, fit_params):
    """
    Compute how Gaussian-like the data is based on the fit parameters.
    Returns a scalar between 0 (bad) and 1 (perfect Gaussian).
    """
    x0, y0, sigma_x, sigma_y, theta = fit_params
    nrows, ncols = data.shape
    y, x = np.mgrid[0:nrows, 0:ncols]

    # Normalize data
    data_norm = data - np.min(data)
    if np.max(data_norm) > 0:
        data_norm = data_norm / np.max(data_norm)

    model = gaussian_2d(x, y, x0, y0, sigma_x, sigma_y, theta, A=1.0, offset=0.0)
    model = model / np.max(model)

    residual = data_norm - model
    score = 1 - np.sum(residual**2) / np.sum(data_norm**2)
    return np.clip(score, 0, 1)

def isolate_main_spot(image, threshold_ratio=0.5):
    """Keep only the brightest blob in the image."""
    # Threshold image
    threshold = threshold_ratio * np.max(image)
    binary = image > threshold

    # Label connected regions
    labels, n = ndimage.label(binary)
    if n == 0:
        return image  # nothing found

    # Find the label with the highest summed intensity
    sums = ndimage.sum(image, labels, range(1, n + 1))
    main_label = np.argmax(sums) + 1

    # Keep only the main region
    mask = labels == main_label
    filtered = image * mask

    return filtered    

def rad2deg(th): return th * 180.0 / np.pi
def deg2rad(th): return th * np.pi / 180.0


def main():
    from pymodaq_gui.utils.utils import mkQApp

    app = mkQApp("Attenuator")

    mainwindow = QtWidgets.QMainWindow()
    dockarea = gutils.DockArea()
    mainwindow.setCentralWidget(dockarea)

    prog = BeamStabilizer(dockarea)
    prog.mainwindow = mainwindow

    app.aboutToQuit.connect(prog.cleanup_threads)

    mainwindow.show()
    app.exec()


if __name__ == '__main__':
    main()
