from setuptools import find_packages, setup

package_name = 'serial_communication_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hhk-laptop',
    maintainer_email='whaihong@g.skku.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'serial_protocol_converter_node = serial_communication_pkg.serial_protocol_converter_node:main',
			'serial_sender_node = serial_communication_pkg.serial_sender_node:main',
        ],
    },
)
