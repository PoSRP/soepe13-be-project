import os
import glob

from ament_index_python.resources import (get_resource, get_resources)

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QModelIndex
from python_qt_binding.QtGui import (QIcon, QStandardItem, QStandardItemModel)
from python_qt_binding.QtWidgets import (QAction, QMenu, QListView, QTreeView, QWidget)

from rclpy import logging

from rosidl_runtime_py.utilities import (get_action, get_service, get_message)
from rosidl_runtime_py import (get_action_interfaces, get_service_interfaces,
                               get_message_interfaces)

from rqt_console.text_browse_dialog import TextBrowseDialog

from rqt_py_common import message_helpers
from rqt_py_common.rqt_roscomm_util import RqtRoscommUtil
from rqt_py_common.message_helpers import (get_action_text_from_class, get_message_text_from_class,
                                           get_service_text_from_class)

from ecat_interfaces.msg import EcatDescription

from .configuration_device_item import ConfigurationDeviceItem
# from configuration_device_item import ConfigurationDeviceItem
from .esi_parser import generate_ros_device_description
from .esi_classes import Description

"""
cb : ComboBox
cbx : CheckBox
btn : Button
tv : TreeView
lv : ListView
le : LineEdit
sb : SpinBox
dsb : DoubleSpinBox

---------------------------

_btn_network_start
_btn_network_stop

_btn_configuration_save
_btn_configuration_reset
_btn_configuration_select
_btn_configuration_delete
_btn_configuration_device_select
_btn_configuration_device_refresh

_tv_configuration_edit : configuration_edit_model
_lv_configuration_list : configuration_list_model
_lv_configuration_devices : configuration_devices_model

"""


class EcatDashboardWidget(QWidget):
    def __init__(self,
                 pkg_name='rqt_ecat_dashboard',
                 ui_filename='ecat_dashboard.ui'):
        super(EcatDashboardWidget, self).__init__()
        self._logger = logging.get_logger('EcatDashboardWidget')
        logging.set_logger_level('EcatDashboardWidget', 10)  # 10 = DEBUG

        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        self._esi_file_dir = os.path.join(package_path, 'share', pkg_name, 'resource')
        self._esi_files = glob.glob(os.path.join(self._esi_file_dir, '*ecat*.xml'))
        self._browsers = []

        self._btn_network_start.clicked.connect(self.btn_network_start_clicked)
        self._btn_network_stop.clicked.connect(self.btn_network_stop_clicked)

        self._btn_configuration_save.clicked.connect(self.btn_configuration_save_clicked)
        self._btn_configuration_reset.clicked.connect(self.btn_configuration_reset_clicked)
        self._btn_configuration_select.clicked.connect(self.btn_configuration_select_clicked)
        self._btn_configuration_delete.clicked.connect(self.btn_configuration_delete_clicked)
        self._btn_configuration_device_select.clicked.connect(
            self.btn_configuration_device_select_clicked)
        self._btn_configuration_device_refresh.clicked.connect(
            self.btn_configuration_device_refresh_clicked)

        self.configuration_edit_model = QStandardItemModel()
        self.configuration_edit_model.setColumnCount(3)
        self.configuration_edit_model.setHorizontalHeaderLabels(['Path', 'Name', 'Value'])
        self._tv_configuration_edit.setModel(self.configuration_edit_model)

        self.configuration_list_model = QStandardItemModel()
        self.configuration_list_model.setColumnCount(1)
        self.configuration_list_model.setHorizontalHeaderLabels(['Configuration Name'])
        self._lv_configuration_list.setModel(self.configuration_list_model)

        self.configuration_devices_model = QStandardItemModel()
        self.configuration_devices_model.setColumnCount(1)
        self.configuration_devices_model.setHorizontalHeaderLabels(['Device Name'])
        self._lv_configuration_devices.setModel(self.configuration_devices_model)
        self.configuration_devices_selected_index = None
        self._lv_configuration_devices.clicked[QModelIndex].connect(
            self.configuration_device_clicked)

    def btn_network_start_clicked(self):
        self._logger.debug("Network start pressed")

    def btn_network_stop_clicked(self):
        self._logger.debug("Network stop pressed")

    def btn_configuration_save_clicked(self):
        self._logger.debug("Configuration save pressed")

    def btn_configuration_reset_clicked(self):
        self._logger.debug("Configuration reset pressed")

    def btn_configuration_select_clicked(self):
        self._logger.debug("Configuration select pressed")

    def btn_configuration_delete_clicked(self):
        self._logger.debug("Configuration delete pressed")

    def configuration_device_clicked(self, index):
        item = self.configuration_devices_model.itemFromIndex(index)
        self._logger.debug("Configuration device selected in list: " + str(item.text()))
        self.configuration_devices_selected_index = index

    def class_to_tree_model(self, obj):
        it = QStandardItem()
        it_idx = 0
        for k in obj.__dict__:
            if '_' in k:
                continue
            if obj.__dict__[k] is None:
                continue
            if isinstance(obj.__dict__[k], str):
                item = QStandardItem()
                item.setData(obj.__dict__[k])
                it.setChild(it_idx, item)
                it_idx += 1
            elif isinstance(obj.__dict__[k], list):
                for sub in obj.__dict__[k]:
                    if isinstance(sub, str):
                        item = QStandardItem()
                        item.setData(sub)
                        it.setChild(it_idx, item)
                    else:
                        item = self.class_to_tree_model(sub)
                        it.setChild(it_idx, item)
                    it_idx += 1
            else:
                item = self.class_to_tree_model(obj.__dict__[k])
                it.setChild(it_idx, item)
                it_idx += 1
        return it

    def btn_configuration_device_select_clicked(self):
        self._logger.debug("Configuration device select pressed")
        if self.configuration_devices_selected_index is None:
            self._logger.warn("No device selected!")
            return False
        item = self.configuration_devices_model.itemFromIndex(
            self.configuration_devices_selected_index)
        self._logger.debug("Selected item: " + item.text())
        self.configuration_edit_model.clear()
        root = self.class_to_tree_model(item)
        self.configuration_edit_model.invisibleRootItem().setChild(0, root)

    def btn_configuration_device_refresh_clicked(self):
        self._logger.debug("Configuration device refresh pressed")
        self.configuration_devices_model.clear()
        for esi_file in self._esi_files:
            self._logger.debug("Looking at: " + str(esi_file))
            device_description = Description(esi_file)
            name = device_description.vendor.name + " " + device_description.groups[0].group_type
            it = ConfigurationDeviceItem(name, device_description)
            self.configuration_devices_model.appendRow(it)

    def cleanup_browsers_on_close(self):
        for browser in self._browsers:
            browser.close()
