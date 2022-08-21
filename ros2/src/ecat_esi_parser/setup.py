from setuptools import setup

package_name = 'ecat_esi_parser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Søren Riisom Pedersen',
    maintainer_email='soepe13@student.sdu.dk',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            package_name + ' = ' + package_name + '.main:main'
        ],
    },
)
