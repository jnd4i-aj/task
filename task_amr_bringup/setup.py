from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_amr_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*.pgm')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aman',
    maintainer_email='aman021jindal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
