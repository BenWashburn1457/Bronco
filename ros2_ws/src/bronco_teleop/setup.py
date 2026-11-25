from setuptools import setup

package_name = 'bronco_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # These two entries let ROS tools find the package after it's installed
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben',
    maintainer_email='benrobertwashburn@gmail.com',
    description='Keyboard (WASD / arrow keys) driving for Bronco, car-style.',
    license='MIT',
    entry_points={
        # "ros2 run bronco_teleop wasd_teleop" runs main() in wasd_teleop.py
        'console_scripts': [
            'wasd_teleop = bronco_teleop.wasd_teleop:main',
        ],
    },
)
