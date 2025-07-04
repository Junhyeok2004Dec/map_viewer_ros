from setuptools import setup

package_name = 'map_viewer_ros'

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
    maintainer='Junhyeok Choe',
    maintainer_email='junhyeok1209@outlook.kr',
    description='Python ROS2 package for robot pose',
    license='MIT',
    tests_require=['pytest'],
)
