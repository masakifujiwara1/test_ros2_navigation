from setuptools import setup
import os
from glob import glob

package_name = 'test_ros2_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools', 'launch', 'gazebo_ros', 'geometry_msgs', 'nav_msgs', 'rclcpp', 'sensor_msgs', 'tf2'],
    zip_safe=True,
    maintainer='fmasa',
    maintainer_email='fmasaki2020@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
