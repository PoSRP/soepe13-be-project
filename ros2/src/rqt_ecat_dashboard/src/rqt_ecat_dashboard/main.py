import sys
from rqt_gui.main import Main


def main():
    plugin = 'rqt_ecat_dashboard.ecat_dashboard.EcatDashboard'
    _main = Main(filename=plugin)
    sys.exit(_main.main(standalone=plugin))


if __name__ == '__main__':
    main()
