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


class EcatDevicesWidget(QWidget):
    def __init__(self,
                 pkg_name='rqt_ecat_dashboard',
                 ui_filename='ecat_devices.ui'):
        super(EcatDevicesWidget, self).__init__()
        self._logger = logging.get_logger('EcatDevicesWidget')

        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        self._esi_file_dir = os.path.join(package_path, 'share', pkg_name, 'resource', 'device-descriptions')
        self._esi_files = glob.glob(os.path.join(self._esi_file_dir, '*'))

        self._browsers = []
