import sys

from rqt_gui.main import Main
from rqt_ecat_dashboard.ecat_dashboard import Dashboard


def main():
  plugin = 'rqt_ecat_dashboard.ecat_dashboard.Dashboard'
  main = Main(filename=plugin)
  sys.exit(main.main(standalone=plugin))


if __name__ == '__main__':
  main()
