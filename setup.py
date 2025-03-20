from setuptools import find_packages, setup

package_name = 'turtlebot3_interplanetar'

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
    maintainer='easy',
    maintainer_email='=',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_gui = turtlebot3_gui.control_gui:main',
            'speaker_node = voice_controlled_turtlebot.speaker_node:main',
            'control_node = voice_controlled_turtlebot.control_node:main',
            'energy_node = voice_controlled_turtlebot.energy_node:main',
        ],
    },
)
