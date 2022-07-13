from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget


class Dashboard(Plugin):
    def __init__(self, context):
        super(Dashboard, self).__init__(context)
        self.setObjectName('Dashboard')
        self._widget = QWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.cleanup_browsers_on_close()

    def save_settings(self, plugin_settings, instance_settings):
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # v = instance_settings.value(k)
        pass
