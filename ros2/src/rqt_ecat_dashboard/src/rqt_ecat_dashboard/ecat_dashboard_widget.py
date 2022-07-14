import os
import glob

from ament_index_python.resources import (get_resource, get_resources)

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import (QIcon, QStandardItem, QStandardItemModel)
from python_qt_binding.QtWidgets import (QAction, QMenu, QTreeView, QWidget)

from rclpy import logging

from rosidl_runtime_py.utilities import (get_action, get_service, get_message)
from rosidl_runtime_py import (get_action_interfaces, get_service_interfaces, get_message_interfaces)

from rqt_console.text_browse_dialog import TextBrowseDialog

from rqt_py_common import message_helpers
from rqt_py_common.rqt_roscomm_util import RqtRoscommUtil
from rqt_py_common.message_helpers import (get_action_text_from_class, get_message_text_from_class,
                                           get_service_text_from_class)

from .ecat_esi_description import Description
from .ecat_devices_widget import EcatDevicesWidget


class EcatDashboardWidget(QWidget):
    def __init__(self,
                 pkg_name='rqt_ecat_dashboard',
                 ui_filename='ecat_dashboard.ui'):
        super(EcatDashboardWidget, self).__init__()
        self._logger = logging.get_logger('EcatDashboardWidget')

        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        self._esi_file_dir = os.path.join(package_path, 'share', pkg_name, 'resource', 'device-descriptions')
        self._esi_files = glob.glob(os.path.join(self._esi_file_dir, '*'))

        self._refresh_devices.setIcon(QIcon.fromTheme('list-add'))
        self._refresh_devices.clicked.connect(self._refresh_devices_clicked)

        self._devices.setIcon(QIcon.fromTheme('list-add'))
        self._devices.clicked.connect(self._devices_clicked)

        device_list_model = QStandardItemModel()
        device_list_model.setColumnCount(1)
        device_list_model.setHorizontalHeaderLabels(['Device Name'])
        self._device_list.setModel(device_list_model)

        self._browsers = []
        self._devices_popup = None

        self._log_write('Hello from the EtherCAT Dashboard Widget')

    def _log_write(self, text: str):
        self._plain_log.appendPlainText(text)

    def _devices_clicked(self):
        self._devices_popup = EcatDevicesWidget()
        self._devices_popup.show()

    def _refresh_devices_clicked(self):
        for esi_file in self._esi_files:
            self._log_write('ESI File: ' + esi_file)

            try:
                device = Description(os.path.join(self._esi_file_dir, esi_file))
                device_str = device.vendor.name + ' - ' + device.groups[0].type

                item = QStandardItem(device_str)
                model = self._device_list.model()
                model.insertRow(0, [item])

            except Exception as ex:
                self._log_write('Caught an exception when reading ESI file: ' + str(ex))

    def cleanup_browsers_on_close(self):
        for browser in self._browsers:
            browser.close()
