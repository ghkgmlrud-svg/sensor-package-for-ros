from setuptools import find_packages, setup

package_name = 'isro_p2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config_0402.txt']),
    ],
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=True,
    maintainer='pi3',
    maintainer_email='pi3@todo.todo',
    description='ROS2 serial driver for ISRO-P2 PIMTP PVA and IMU data',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'isro_p2_node = isro_p2_pkg.isro_p2_node:main',
        ],
    },
)
