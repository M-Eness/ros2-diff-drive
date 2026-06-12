from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch klasöründeki tüm .py dosyalarını share/paket_adi/launch içine at
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # urdf klasöründeki tüm .urdf dosyalarını share/paket_adi/urdf içine at
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),

        # config klasöründeki her şeyi (.yaml, .json), share/paket_ismi/config içine kopyala:
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # ===============================

        # "worlds klasöründeki her şeyi (.sdf), share/paket_ismi/worlds içine kopyala" der:
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),

        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.test_node:main",
            "simple_subscriber = my_robot_controller.simple_subscriber:main",
            "robot_driver = my_robot_controller.robot_driver:main",
            "mission_manager = my_robot_controller.mission_manager:main",
            'ackermann_bridge = my_robot_controller.ackermann_bridge:main',
            'vehicle_emulator = my_robot_controller.vehicle_emulator:main',
            'traffic_sign_detector = my_robot_controller.traffic_sign_detector:main',
        ],
    },
)