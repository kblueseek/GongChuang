# setup.py：告诉 colcon（ROS2 的构建工具）"这个包怎么安装"。
#
# 大白话：colcon 构建时，会执行这个文件，把包里的文件"安装"到 install/lidar_bringup/ 下。
# ROS2 运行时用的是 install/ 里的东西，不是 src/ 里的。

import os
from glob import glob            # glob：按通配符匹配文件（比如 launch/*.launch.py）
from setuptools import setup     # Python 打包的标准工具

package_name = 'lidar_bringup'   # 包名，必须和目录名、package.xml 里的 name 一致

setup(
    name=package_name,
    version='0.1.0',
    packages=[],   # 这个包没有 Python 模块（只有 launch 和 config），所以 packages 留空。

    # data_files = 要复制安装到 install/ 的文件清单。每项是 (目标目录, [源文件...])。
    data_files=[
        # 这一项是"包标记"：在 ament 的资源索引里放一个空文件，让 ros2 能发现这个包。
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml 也装过去
        ('share/' + package_name, ['package.xml']),
        # 把所有 launch/*.launch.py 装到 share/lidar_bringup/launch/，
        # 这样 ros2 launch lidar_bringup xxx.launch.py 才找得到。
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 把所有 config/*.yaml 装到 share/lidar_bringup/config/
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kblue',
    maintainer_email='kblueseek@users.noreply.github.com',
    description='YDLIDAR X3 PRO bringup (launch + params)',
    license='MIT',
)
