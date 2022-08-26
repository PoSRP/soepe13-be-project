from qt_gui.plugin import Plugin
from .ecat_dashboard_widget import EcatDashboardWidget


class EcatDashboard(Plugin):
    def __init__(self, context):
        super(EcatDashboard, self).__init__(context)
        self.setObjectName('EcatDashboard')

        assert hasattr(context, 'node'), 'Context does not have a node.'
        self._widget = EcatDashboardWidget(context.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # self._widget.cleanup_browsers_on_close()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # v = instance_settings.value(k)
        pass
