from setuptools import find_packages, setup

package_name = 'lidar_perception_pkg'

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
            'lidar_publisher_node = lidar_perception_pkg.lidar_publisher_node:main',
			'lidar_processor_node = lidar_perception_pkg.lidar_processor_node:main',
			'lidar_obstacle_detector_node = lidar_perception_pkg.lidar_obstacle_detector_node:main',
			
        ],
    },
)
