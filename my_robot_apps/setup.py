from setuptools import find_packages, setup

package_name = 'my_robot_apps'

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
    maintainer='yhs',
    maintainer_email='yhs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_local_node = my_robot_apps.gesture_local_node:main',
            'turn_to_person_node = my_robot_apps.turn_to_person_node:main',
            'depth_distance_node = my_robot_apps.depth_distance_node:main',
            'follow_by_distance_node = my_robot_apps.follow_by_distance_node:main',
            'come_mode_node = my_robot_apps.come_mode_node:main',
            
        ],
    },
)
