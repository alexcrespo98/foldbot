from setuptools import setup

package_name = 'foldbot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
    ('share/' + package_name + '/launch', ['launch/foldbot.launch.py']),
    ('share/' + package_name + '/config', ['config/params.yaml']),
    ('share/' + package_name, ['package.xml']),
    ('share/ament_index/resource_index/packages', ['resource/foldbot']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex crespo',
    maintainer_email='alexcrespo12345@gmail.com',
    description='Napkin-folding robot system with sensor-based control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_controller = foldbot.main_controller:main',
            'gantry_controller = foldbot.gantry_controller:main',
            'servo_controller = foldbot.servo_controller:main',
            'solenoid_controller = foldbot.solenoid_controller:main',
            'vacuum_controller = foldbot.vacuum_controller:main',
            'left_limit_switch = foldbot.left_limit_switch:main',
            'right_limit_switch = foldbot.right_limit_switch:main',
            'napkin_sensor = foldbot.napkin_sensor:main',
        ],
    },
)

