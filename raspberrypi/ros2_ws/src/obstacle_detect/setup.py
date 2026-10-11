# setup.py：告诉 colcon 怎么安装 obstacle_detect 包。
# 和 lidar_bringup 的区别：这个包里有真正的 Python 代码（节点），所以有 packages 和 entry_points。

from setuptools import find_packages, setup   # find_packages：自动找到所有 Python 包（带 __init__.py 的目录）

package_name = 'obstacle_detect'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),   # 自动把 obstacle_detect/ 这个包打进去
    data_files=[
        # 包标记 + package.xml，和 lidar_bringup 一样
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kblue',
    maintainer_email='kblueseek@users.noreply.github.com',
    description='扇区障碍检测',
    license='MIT',
    # entry_points 是关键：它把"一个 Python 函数"变成一个"可以直接运行的命令"。
    # 左边 obstacle_detect_node = 命令名（ros2 run obstacle_detect obstacle_detect_node 里的后一个词），
    # 右边 obstacle_detect.obstacle_detect_node:main = 模块.文件 : 函数名。
    # colcon 安装后会自动生成一个同名可执行文件，去调用那个 main() 函数。
    entry_points={
        'console_scripts': [
            'obstacle_detect_node = obstacle_detect.obstacle_detect_node:main',
        ],
    },
)
