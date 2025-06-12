from setuptools import find_packages, setup

package_name = 'camera_perception_pkg'

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
    maintainer='hhk',
    maintainer_email='whaihong@g.skku.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher_node = camera_perception_pkg.image_publisher_node:main',
            'yolov8_node = camera_perception_pkg.yolov8_node:main',
            'traffic_light_detector_node = camera_perception_pkg.traffic_light_detector_node:main',
            'lane_info_extractor_node = camera_perception_pkg.lane_info_extractor_node:main',
        ],
    },
)
