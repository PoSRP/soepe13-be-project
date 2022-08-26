import os
import glob

from python_qt_binding import (loadUi)
from python_qt_binding.QtCore import (Qt)
from python_qt_binding.QtGui import (QIcon, QStandardItem, QStandardItemModel, QPixmap)
from python_qt_binding.QtWidgets import (QAction, QMenu, QTreeView, QWidget, QStatusBar, QLabel)

import rclpy
from ament_index_python.resources import (get_resource, get_resources)
from rqt_py_common.message_helpers import (get_service_class, SRV_MODE)


class Icons:
    def __init__(self, package_path, pkg_name):
        self.width = 16
        self.height = self.width
        self.dot_green = QPixmap(os.path.join(package_path, 'share', pkg_name, 'resource',
                                              'green-dot.png'))
        self.dot_yellow = QPixmap(os.path.join(package_path, 'share', pkg_name, 'resource',
                                               'yellow-dot.png'))
        self.dot_orange = QPixmap(os.path.join(package_path, 'share', pkg_name, 'resource',
                                               'orange-dot.png'))
        self.dot_red = QPixmap(os.path.join(package_path, 'share', pkg_name, 'resource',
                                            'red-dot.png'))

        self.dot_green = self.dot_green.scaled(self.width, self.height, Qt.KeepAspectRatio,
                                               Qt.FastTransformation)
        self.dot_yellow = self.dot_yellow.scaled(self.width, self.height, Qt.KeepAspectRatio,
                                                 Qt.FastTransformation)
        self.dot_orange = self.dot_orange.scaled(self.width, self.height, Qt.KeepAspectRatio,
                                                 Qt.FastTransformation)
        self.dot_red = self.dot_red.scaled(self.width, self.height, Qt.KeepAspectRatio,
                                           Qt.FastTransformation)


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

    def btn_parameter_load_clicked(self):
        self.log('Parameter load clicked')

    def btn_parameter_set_clicked(self):
        self.log('Parameter set clicked')

    def btn_network_services_refresh_clicked(self):
        self.log('Network services refresh clicked')
        service_names_and_types = self._node.get_service_names_and_types()
        self._services = {}
        self._masters = {}
        for service_name, service_types in service_names_and_types:
            if 'ecat_server' not in service_name:
                continue
            else:
                node_name = str(service_name).split('/')[1]
                self._masters[node_name] = ''
            if len(service_types) > 1:
                continue
            service_name_tokens = service_types[0].split('/')
            if len(service_name_tokens) == 3 and service_name_tokens[1] != SRV_MODE:
                continue
            service_class = get_service_class(service_types[0])
            if service_class is not None:
                self._services[service_name] = service_types[0]

        self._cb_masters.clear()
        if len(self._masters.keys()):
            self.log('Found master nodes: ' + str([str(k) for k in self._masters.keys()]))
            self._cb_masters.addItems(sorted(self._masters.keys()))
            self._
        else:
            self.log('No active master nodes were found!')

    def btn_network_start_clicked(self):
        self.log('Network start clicked')
        srv = '/' + self._cb_masters.currentText() + '/start_network'
        self._call_master_node_service(srv)

    def btn_network_stop_clicked(self):
        self.log('Network stop clicked')
        srv = '/' + self._cb_masters.currentText() + '/stop_network'
        self._call_master_node_service(srv)

    def _call_master_node_service(self, service_name):
        _service_info = {}
        _service_info['service_name'] = service_name
        try:
            _service_info['service_class_name'] = self._services[service_name]
        except:
            self.log('Network starting service is missing, try refreshing the server list')
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
        while rclpy.ok() and not future.done():
            pass
        if future.result() is not None:
            response = future.result()
            self.log(str(response))
        else:
            self.log('Error calling network start service')
        self._node.destroy_client(cli)

    def log(self, text: str):
        self._te_log.append(text)
        self._logger.debug(text)
