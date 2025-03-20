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
            'control_gui = turtlebot3_interplanetar.control_gui:main',
            'speaker_node = turtlebot3_interplanetar.speaker_node:main',
            'speaker_node_text = turtlebot3_interplanetar.speaker_node_text:main',
            'control_node = turtlebot3_interplanetar.control_node:main',
            'energy_node = turtlebot3_interplanetar.energy_node:main',
        ],
    },
)
