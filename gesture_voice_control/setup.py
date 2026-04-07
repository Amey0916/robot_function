from setuptools import find_packages, setup

package_name = 'gesture_voice_control'

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
            'voice_recognition = gesture_voice_control.voice_recognition_node:main',
            'device_control = gesture_voice_control.device_control_node:main',
            'gesture_body_detection_node = gesture_voice_control.gesture_body_detection_node:main',
        ],
    },
)
