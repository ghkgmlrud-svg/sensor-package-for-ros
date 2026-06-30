from setuptools import find_packages, setup

package_name = 'fusion_sensor_pkg'

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
    maintainer='pi3',
    maintainer_email='pi3@todo.todo',
    description='ROS2 fusion driver for MW-AHRS-X1 and ISRO-P2 sensors',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fusion_sensor_node = fusion_sensor_pkg.fusion_sensor_node:main',
        ],
    },
)
