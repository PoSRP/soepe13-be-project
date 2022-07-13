from setuptools import setup

package_name = 'rqt_ecat_dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource',
            ['resource/ecat_dashboard.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Søren Riisom Pedersen',
    maintainer_email='soepe13@student.sdu.dk',
    description=(
        'A Python GUI plugin for controlling the EtherCAT Master Server node.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            package_name + ' = ' + package_name + '.main:main'
        ],
    },
)
