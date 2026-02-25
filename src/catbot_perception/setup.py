from setuptools import find_packages, setup

package_name = 'catbot_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensors.launch.py']),
    ],
    install_requires=['setuptools', 'pyyaml', 'pygame'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='catbot_perception package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'my_node = catbot_perception.my_node:main',
            'perceive = catbot_perception.perceive:main',
            'imu_node = catbot_perception.sensors.imu:main',
            'encoder_node = catbot_perception.sensors.encoders:main',
            'command_node = catbot_perception.sensors.commands:main',
        ],
    },
)
