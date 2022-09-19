import os
import glob
import time
from datetime import datetime

from python_qt_binding import (loadUi)
from python_qt_binding.QtCore import (Qt)
from python_qt_binding.QtGui import (QIcon, QStandardItem, QStandardItemModel, QPixmap)
from python_qt_binding.QtWidgets import (QAction, QMenu, QTreeView, QWidget, QStatusBar, QLabel,
                                         QTableWidgetItem, QMessageBox)

import rclpy
from rclpy.action import ActionClient
from ament_index_python.resources import (get_resource, get_resources)
from rqt_py_common.message_helpers import (get_service_class, SRV_MODE)
from action_msgs.msg import GoalStatus

from ecat_interfaces.action import ExecuteMove

import pyqtgraph as pg


class EcatDashboardWidget(QWidget):
    def __init__(self, node,
                 pkg_name='rqt_ecat_dashboard',
                 ui_filename='ecat_dashboard.ui'):
        super(EcatDashboardWidget, self).__init__()
        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        self._logger = rclpy.logging.get_logger('EcatDashboardWidget')
        self._icons = Icons(package_path, pkg_name)
        self._node = node
        self._service_info = {}
        self._services = {}
        self._masters = {}

        self._lbl_sb_master.setText('N/A')
        self._lbl_sb_master_status.setText('N/A')
        self._lbl_sb_slave.setText('N/A')
        self._lbl_sb_slave_status.setText('N/A')
        self._lbl_sb_slave_mode.setText('N/A')
        self._lbl_sb_slave_mode_value.setText('00.00')
        self._lbl_sb_master_status_icon.setPixmap(self._icons.dot_red)
        self._lbl_sb_slave_status_icon.setPixmap(self._icons.dot_red)

        self._btn_network_start.clicked.connect(self.btn_network_start_clicked)
        self._btn_network_stop.clicked.connect(self.btn_network_stop_clicked)
        self._btn_network_services_refresh.clicked.connect(
            self.btn_network_services_refresh_clicked)

        self._btn_parameter_load.clicked.connect(self.btn_parameter_load_clicked)
        self._btn_parameter_set.clicked.connect(self.btn_parameter_set_clicked)

        self._btn_point_add.clicked.connect(self.btn_point_add_clicked)
        self._btn_point_remove.clicked.connect(self.btn_point_remove_clicked)
        self._btn_point_clear_all.clicked.connect(self.btn_point_clear_all_clicked)

        self._btn_point_profile_save.clicked.connect(self.btn_point_profile_save_clicked)
        self._btn_point_profile_load.clicked.connect(self.btn_point_profile_load_clicked)
        self._btn_point_profile_delete.clicked.connect(self.btn_point_profile_delete_clicked)

        self._btn_move_execute.clicked.connect(self.btn_move_execute_clicked)
        self._btn_move_stop.clicked.connect(self.btn_move_stop_clicked)

        self._points_model = QStandardItemModel()
        self._points_model.setColumnCount(2)
        self._points_model.setHorizontalHeaderLabels(['Time [s]', 'Position [mm]'])
        self._points_selection_store = []
        self._tv_points.setModel(self._points_model)
        self._points_model.itemChanged.connect(self.tv_point_item_edited)
        self._tv_points.selectionModel().selectionChanged.connect(self.tv_point_item_selected)

        self._graph_widget = pg.PlotWidget()
        self._graph_widget.setBackground('#dddddd')
        self._graph_widget.showGrid(x=True, y=True)
        self._graph_top_layout.addWidget(self._graph_widget)
        self._graph_lines = {'planned': None, 'achieved': None}
        self.graph_mode('position')

        self._point_profile_store_path = os.path.join(os.path.expanduser('~'), '.local',
                                                      'ecat_dashboard', 'point_profiles')
        self._point_profile_types = {
            'active': 'position',
            'position': {
                'file_extension': '.pprofile',
                'capitalized': 'Position',
                'unit': 'mm',
                'move_mode_index': 0
            },
            'velocity': {
                'file_extension': '.vprofile',
                'capitalized': 'Velocity',
                'unit': 'mm/s',
                'move_mode_index': 1
            }
        }
        self._point_profiles_model = QStandardItemModel()
        self._point_profiles_model.setHorizontalHeaderLabels(['Point profiles'])
        self._lv_point_profiles.setModel(self._point_profiles_model)
        self._lv_point_profiles.selectionModel().selectionChanged.connect(
            self.lv_point_profiles_item_selected)
        self.refresh_point_profiles()

        self._action_client = None
        self._action_future = None
        self._action_goal = None
        self._action_feedback_model = QStandardItemModel()
        self._action_feedback_model.setColumnCount(2)
        self._action_feedback_model.setHorizontalHeaderLabels(['Time [s]', 'Position [mm]'])
        self._tv_action_feedback.setModel(self._action_feedback_model)

        self._cb_move_mode.addItem('Position mode')
        self._cb_move_mode.addItem('Velocity mode')
        self._cb_move_mode.currentIndexChanged.connect(self.cb_move_mode_index_changed)

    def cb_move_mode_index_changed(self):
        text = self._cb_move_mode.currentText()
        mode = text.split(' ')[0].lower()
        self._point_profile_types['active'] = mode
        mode_header = self._point_profile_types[mode]['capitalized'] + \
            ' [' + self._point_profile_types[mode]['unit'] + ']'
        self._lbl_point_mode.setText(mode_header)
        self._points_model.setHorizontalHeaderLabels(['Time [s]', mode_header])
        self._action_feedback_model.clear()
        self._action_feedback_model.setColumnCount(2)
        self._action_feedback_model.setHorizontalHeaderLabels(['Time [s]', mode_header])
        self.graph_mode(mode)
        self.log('Move mode set to: ' + mode)

    def handle_service_result_start_network(self, result):
        self.log('Network start result: ' + str(result))

    def handle_service_result_stop_network(self, result):
        self.log('Network stop result: ' + str(result))

    def handle_action_feedback_execute_move(self, feedback_msg):
        time_data = feedback_msg.feedback.time_data
        value_data = feedback_msg.feedback.value_data
        for index in range(len(time_data)):
            self._action_feedback_model.appendRow([FloatStandardItem(str(round(time_data[index], 5))),
                                                   FloatStandardItem(str(round(value_data[index], 5)))])
            self.log(f'Received feedback: ({time_data[index]}, {value_data[index]})')

    def handle_action_result_execute_move(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.log('Move succeeded! Result: {0}'.format(result.msg))
        else:
            self.log('Move failed with status: {0}'.format(status))
        self._action_future = None
        self._action_client = None
        self._action_goal = None

    def handle_action_goal_response_execute_move(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log('Move rejected')
            self._action_future = None
            self._action_client = None
            self._action_goal = None
            return
        self._action_goal = goal_handle
        self.log('Move accepted')
        self._action_future = goal_handle.get_result_async()
        self._action_future.add_done_callback(self.handle_action_result_execute_move)

    def convert_profile_to_point_list(self, times, values, interval_us=1000, frequency=1000):
        # Frequency and interval will behave as expected, server rate (1000Hz) is not set here
        # The server will eventually host a parameter for setting it
        interval_us = 1000
        # if frequency != 1000:
        #     interval_us = int(round(1000000 / frequency, 0))

        result_times = []
        result_values = []
        input_idx = 0

        while input_idx + 1 < len(times):
            t0, v0 = times[input_idx] * 1000000, values[input_idx]
            t1, v1 = times[input_idx + 1] * 1000000, values[input_idx + 1]
            input_idx += 1
            dv = v1 - v0
            dt = t1 - t0
            time_index = 0
            
            while time_index <= dt:
                # Skip if time point was calculated using previous values
                if len(result_times) and float((time_index + t0)) / 1000000 == result_times[-1]:
                    time_index += interval_us
                    continue
                
                if dv == 0:
                    result_values.append(float(v0))
                else:
                    result_values.append(float((dv / dt) * time_index + v0))
                    
                result_times.append(float((time_index + t0) / 1000000))
                time_index += interval_us
                
                self.log(f'Point calculated: ({result_times[-1]}, {result_values[-1]})')
        
        self.log('Done calculating points')
        return result_times, result_values

    def btn_move_execute_clicked(self):
        if self._action_client is not None:
            self.log('The action client is already active')
            return
        # if self._cb_slave_mode.currentText() != self._cb_move_mode.currentText():
        #     self.log('Current move mode does not match with slave mode')
        #     return
        # TODO: Check if the server is active, abort if not

        point_times = []
        point_values = []
        for index in range(self._points_model.rowCount()):
            row = [self._points_model.item(index, 0),
                   self._points_model.item(index, 1)]
            point_times.append(float(row[0].text()))
            point_values.append(float(row[1].text()))

        time_data, value_data = self.convert_profile_to_point_list(point_times, point_values)
        self._action_client = ActionClient(self._node, ExecuteMove, 'execute_move')
        goal_msg = ExecuteMove.Goal()
        goal_msg.time_data_ns = [float(i * 10**9 ) for i in time_data]  # Multiply up to seconds
        goal_msg.position_data_um = [float(i * 10**3) for i in value_data]  # Multiply up to millimeters

        self._action_client.wait_for_server()
        self._action_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.handle_action_feedback_execute_move)
        self._action_future.add_done_callback(self.handle_action_goal_response_execute_move)
        self.cb_move_mode_index_changed()

    def move_cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.log('Move successfully canceled')
        else:
            self.log('Move failed to cancel')

    def btn_move_stop_clicked(self):
        if self._action_future is None:
            self.log('No active action to stop')
        future = self._action_goal.cancel_goal_async()
        future.add_done_callback(self.move_cancel_done)

    def lv_point_profiles_item_selected(self):
        # Selections are constrained to a single row
        index = self._lv_point_profiles.selectionModel().selectedRows()
        item = self._point_profiles_model.item(index[0].row(), 0)
        self._le_point_profile_save_name.setText(item.text().split('.')[0])

    def refresh_point_profiles(self):
        if not os.path.isdir(self._point_profile_store_path):
            os.makedirs(self._point_profile_store_path)
        profiles = os.listdir(self._point_profile_store_path)
        self._point_profiles_model.clear()
        for profile in profiles:
            self.log(f"Found profile: {profile}")
            self._point_profiles_model.appendRow(QStandardItem(profile))

    def btn_point_profile_save_clicked(self):
        time_data = []
        mode_data = []
        for index in range(self._points_model.rowCount()):
            row = [self._points_model.item(index, 0),
                   self._points_model.item(index, 1)]
            time_data.append(float(row[0].text()))
            mode_data.append(float(row[1].text()))
        active_mode = self._point_profile_types['active']
        extension = self._point_profile_types[active_mode]['file_extension']
        name = self._le_point_profile_save_name.text() + extension
        path = os.path.join(self._point_profile_store_path, name)
        try:
            with open(path, 'w') as f:
                for index in range(len(time_data)):
                    f.write(f'{time_data[index]},{mode_data[index]}\n')
            self.log(f"Saved profile: {name}")
            self.refresh_point_profiles()
        except Exception as ex:
            self.log(f"Could not save profile "
                     f"'{name}': {str(ex)}")
            if os.path.exists(path):
                os.remove(path)

    def btn_point_profile_load_clicked(self):
        index = self._lv_point_profiles.selectionModel().selectedRows()
        if not len(index):
            self.log('No point profile selected')
            return
        name = self._point_profiles_model.itemFromIndex(index[0]).text()

        path = os.path.join(self._point_profile_store_path, name)
        if not os.path.exists(path):
            self.log('Selected profile path does not exist: ' + name)
            return

        try:
            with open(path) as f:
                lines = f.readlines()
        except:
            self.log('Could not open profile: ' + name)
            return

        extension = '.' + name.split('.')[-1]
        if extension == self._point_profile_types['position']['file_extension']:
            self._cb_move_mode.setCurrentIndex(
                self._point_profile_types['position']['move_mode_index'])
        elif extension == self._point_profile_types['velocity']['file_extension']:
            self._cb_move_mode.setCurrentIndex(
                self._point_profile_types['velocity']['move_mode_index'])
        else:
            self.log("Unrecognized extension in 'btn_point_profile_load_clicked': " + extension)
            return

        self._points_model.clear()
        self._points_model.setColumnCount(2)
        self.cb_move_mode_index_changed()

        for line in lines:
            time, value = line.split(',')
            self._points_model.appendRow([FloatStandardItem(str(time)),
                                          FloatStandardItem(str(value))])
        # self._points_model.layoutChanged.emit()
        self._tv_points.sortByColumn(0, Qt.AscendingOrder)
        self.update_planned_graph()
        self.log('Profile loaded: ' + name)

    def btn_point_profile_delete_clicked(self):
        reply = QMessageBox.question(self, 'Delete profile',
                                     'Are you sure you want to delete the point profile?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.No:
            self.log('Delete profile declined')
            return
        index = self._lv_point_profiles.selectionModel().selectedRows()
        if not len(index):
            self.log('No point profile selected')
            return
        name = self._point_profiles_model.itemFromIndex(index[0]).text()
        path = os.path.join(self._point_profile_store_path, name)
        if os.path.exists(path):
            os.remove(path)
            self.log('Deleted profile: ' + name)
            self.refresh_point_profiles()
        else:
            self.log('Profile file does not exist, nothing deleted')

    def tv_point_item_selected(self, item):
        # Selections are constrained to a single row
        index = self._tv_points.selectionModel().selectedRows()
        p0, p1 = (self._points_model.item(index[0].row(), 0),
                  self._points_model.item(index[0].row(), 1))
        self._points_selection_store = [p0.text(), p1.text()]

    def tv_point_item_edited(self, item):
        # Selections are constrained to a single row
        index = self._tv_points.selectionModel().selectedRows()
        if not len(index):
            self.log('No point selected')
            return
        p0, p1 = (self._points_model.item(index[0].row(), 0),
                  self._points_model.item(index[0].row(), 1))
        try:
            x = float(p0.text())
            y = float(p1.text())
            if self._points_selection_store[0] != p0.text() or \
                    self._points_selection_store[1] != p1.text():
                self.log(f'Point value change from ({p0.text()}, {p1.text()}) to ({x}, {y}) on row'
                         f' {index[0].row()}')
        except:
            self.log(f"Rejecting value change, '{item.text()}' is not convertible to float")
            p0.setText(self._points_selection_store[0])
            p1.setText(self._points_selection_store[1])
            return
        self._points_model.layoutChanged.emit()
        self._tv_points.sortByColumn(0, Qt.AscendingOrder)
        self.update_planned_graph()

    def btn_point_add_clicked(self):
        vx = self._le_point_mode_value.text()
        vy = self._le_point_time_value.text()
        try:
            vx = float(vx)
            vy = float(vy)
        except:
            self.log('Could not convert values for new point to float')
            if vx == vy == '':
                self.log('Fields are empty, inserting default (0, 0) point')
                vx = vy = 0.0
            else:
                return
        self._points_model.appendRow([FloatStandardItem(str(round(vy, 5))),
                                      FloatStandardItem(str(round(vx, 5)))])
        self._points_model.layoutChanged.emit()
        self._tv_points.sortByColumn(0, Qt.AscendingOrder)
        self.update_planned_graph()
        self.log(f'Added point ({vy}, {vx})')

    def btn_point_remove_clicked(self):
        if None in self._points_selection_store:
            self.log('No selection active in point list, nothing removed')
            return
        # Selections are constrained to a single row
        index = self._tv_points.selectionModel().selectedRows()
        if not len(index):
            self.log('No point selected')
            return
        p0, p1 = (self._points_model.item(index[0].row(), 0),
                  self._points_model.item(index[0].row(), 1))
        self._points_model.takeRow(index[0].row())
        self.update_planned_graph()
        self.log(f'Removed point ({p0.text()}, {p1.text()})')

    def btn_point_clear_all_clicked(self):
        reply = QMessageBox.question(self, 'Clear points', 'Are you sure you want to clear the '
                                                           'points list?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.No:
            self.log('Clear all points declined')
            return
        self._points_model.clear()
        self._points_model.setColumnCount(2)
        mode = self._point_profile_types['active']
        mode_header = self._point_profile_types[mode]['capitalized'] + \
            ' [' + self._point_profile_types[mode]['unit'] + ']'
        self._lbl_point_mode.setText(mode_header)
        self._points_model.setHorizontalHeaderLabels(['Time [s]', mode_header])
        self.update_planned_graph()
        self.log('Cleared all points')

    def update_planned_graph(self):
        time_data = []
        mode_data = []
        for index in range(self._points_model.rowCount()):
            row = [self._points_model.item(index, 0),
                   self._points_model.item(index, 1)]
            time_data.append(float(row[0].text()))
            mode_data.append(float(row[1].text()))
        self.graph_data(time_data, mode_data)

    def graph_mode(self, mode='position'):
        if mode not in ['position', 'velocity']:
            self.log('Mode passed is invalid: ' + str(mode))
            return
        # self._graph_widget.clear()
        self._graph_widget.addLegend()
        styles = {'color': '#000000', 'font-size': '16px'}
        self._graph_widget.setLabel('bottom', 'Time (s)', **styles)
        if mode == 'position':
            self._points_model.setHorizontalHeaderLabels(['Time [s]', 'Position [mm]'])
            self._graph_widget.setLabel('left', 'Position (mm)', **styles)
        else:
            self._points_model.setHorizontalHeaderLabels(['Time [s]', 'Velocity [mm/s]'])
            self._graph_widget.setLabel('left', 'Velocity (mm/s)', **styles)

    def graph_data(self, datax, datay, position='planned'):
        if position not in ['planned', 'achieved']:
            self.log('Position passed is invalid: ' + str(position))
            return
        if position == 'planned':
            if self._graph_lines['planned'] is None:
                pen = pg.mkPen(color=(0, 0, 255), width=3, style=Qt.DashLine)
                line = self._graph_widget.plot(datax, datay, name='planned', pen=pen,
                                               symbol='+', symbolSize=16, symbolBrush='#ff9911')
                self._graph_lines['planned'] = line
            else:
                self._graph_lines['planned'].setData(datax, datay)
        else:
            if self._graph_lines['achieved'] is None:
                pen = pg.mkPen(color=(255, 0, 0), width=3, style=Qt.DashLine)
                line = self._graph_widget.plot(datax, datay, name='achieved', pen=pen)
                self._graph_lines['achieved'] = line
            else:
                self._graph_lines['achieved'].setData(datax, datay)

    def btn_parameter_load_clicked(self):
        pass

    def btn_parameter_set_clicked(self):
        pass

    def btn_network_services_refresh_clicked(self):
        service_names_and_types = self._node.get_service_names_and_types()
        self._services = {}
        self._masters = {}
        for service_name, service_types in service_names_and_types:
            if 'ecat_server' not in service_name:
                continue
            if len(service_types) > 1:
                continue
            service_name_tokens = service_types[0].split('/')
            if len(service_name_tokens) == 3 and service_name_tokens[1] != SRV_MODE:
                continue
            node_name = str(service_name).split('/')[1]
            self._masters[node_name] = ''
            service_class = get_service_class(service_types[0])
            if service_class is not None:
                self._services[service_name] = service_types[0]

        self._cb_masters.clear()
        if len(self._masters.keys()):
            self.log('Found master nodes: ' + str([str(k) for k in self._masters.keys()]))
            self._cb_masters.addItems(sorted(self._masters.keys()))
        else:
            self.log('No active master nodes were found!')

    def btn_network_start_clicked(self):
        srv = '/' + self._cb_masters.currentText() + '/start_network'
        self._call_master_node_service(srv)

    def btn_network_stop_clicked(self):
        srv = '/' + self._cb_masters.currentText() + '/stop_network'
        self._call_master_node_service(srv)

    def _call_master_node_service(self, service_name):
        self.log('Not implemented')
        return

        _service_info = {}
        _service_info['service_name'] = service_name
        try:
            _service_info['service_class_name'] = self._services[service_name]
        except:
            self.log(f'Service {service_name} is missing')
            return
        service_class = get_service_class(self._service_info['service_class_name'])
        assert service_class, 'Could not find class {} for service: {}'.format(
            self._services[service_name], service_name)
        _service_info['service_class'] = service_class
        _service_info['expressions'] = {}
        _service_info['counter'] = 0
        current_services = dict(self._node.get_service_names_and_types())
        if self._service_info['service_name'] not in current_services:
            self.log('Service not available: ' + self._service_info['service_name'])
            return
        request = self._service_info['service_class'].Request()
        self.fill_message_slots(
            request, self._service_info['service_name'], self._service_info['expressions'],
            self._service_info['counter'])
        cli = self._node.create_client(
            self._service_info['service_class'], self._service_info['service_name'])
        future = cli.call_async(request)

        # TODO: Add callback to future and destroy cli there

        self._active_service_futures[service_name] = RosCallFuture(cli, future)
        while rclpy.ok() and not future.done():
            pass
        if future.result() is not None:
            response = future.result()
            self.log(str(response))
        else:
            self.log('Error calling network service: ' + service_name)
        self._node.destroy_client(cli)

    def log(self, text: str):
        dt = datetime.fromtimestamp(time.time())
        sdt = dt.strftime('%H:%M:%S')
        self._te_log.append(sdt + ': ' + text)
        self._logger.info(text)


class FloatStandardItem(QStandardItem):
    def __lt__(self, other):
        return float(self.text()) < float(other.text())


class Icons:
    def __init__(self, package_path, pkg_name):
        self.width = 16
        self.height = self.width
        path = os.path.join(package_path, 'share', pkg_name, 'resource')
        self.dot_green = QPixmap(os.path.join(path, 'green-dot.png'))
        self.dot_yellow = QPixmap(os.path.join(path, 'yellow-dot.png'))
        self.dot_orange = QPixmap(os.path.join(path, 'orange-dot.png'))
        self.dot_red = QPixmap(os.path.join(path, 'red-dot.png'))
        self.dot_green = self.dot_green.scaled(self.width, self.height, Qt.KeepAspectRatio,
                                               Qt.FastTransformation)
        self.dot_yellow = self.dot_yellow.scaled(self.width, self.height, Qt.KeepAspectRatio,
                                                 Qt.FastTransformation)
        self.dot_orange = self.dot_orange.scaled(self.width, self.height, Qt.KeepAspectRatio,
                                                 Qt.FastTransformation)
        self.dot_red = self.dot_red.scaled(self.width, self.height, Qt.KeepAspectRatio,
                                           Qt.FastTransformation)


class ExecuteMoveActionClient:
    # TODO: Create and import the execute move action type
    def __init__(self, node):
        self._node = node
        self._client = ActionClient(node, None, 'execute_move')
        self._future = None

    def __del__(self):
        self._node.destroy_client(self._client)

    def send_goal(self, planned_move):
        goal_msg = None
        goal_msg.mode = planned_move['mode']
        goal_msg.data = planned_move['data']
        self._client.wait_for_server()
        self._future = self._client.send_goal_async(goal_msg,
                                                    feedback_callback=self.feedback_callback)
        self._future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self._future = goal_handle.get_result_async()
        self._future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
