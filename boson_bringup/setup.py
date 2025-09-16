import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'boson_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvh',
    maintainer_email='nvh@todo.todo',
    description='Launch files for the boson camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
)