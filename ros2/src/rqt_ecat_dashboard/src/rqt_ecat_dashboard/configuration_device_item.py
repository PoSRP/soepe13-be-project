from python_qt_binding.QtGui import QStandardItem
from ecat_interfaces.msg import EcatDescription


class ConfigurationDeviceItem(QStandardItem):
    description = None

    def __init__(self, name, description):
        super(ConfigurationDeviceItem, self).__init__()
        self.setText(name)
        self.description = description
