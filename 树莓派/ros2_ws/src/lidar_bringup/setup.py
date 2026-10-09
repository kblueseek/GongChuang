import os
from glob import glob

from setuptools import setup

package_name = 'lidar_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],  # 纯 launch/config 包，无 Python 代码
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kblue',
    maintainer_email='kblueseek@users.noreply.github.com',
    description='YDLIDAR X3 PRO bringup (launch + params)',
    license='MIT',
)
