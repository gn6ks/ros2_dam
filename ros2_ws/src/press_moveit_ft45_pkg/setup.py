from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'press_moveit_ft45_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='pablo@email.com',
    description='Demo MoveIt2 con iiwa7',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # nombre_ejecutable = nombre_paquete.nombre_archivo:funcion_main
            'press_motion_node = press_moveit_ft45_pkg.press_motion_node:main',
        ],
    },
)