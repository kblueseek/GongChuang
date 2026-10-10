# setup.py：告诉 colcon 怎么安装 f407_bridge 包。
# 和 obstacle_detect 结构完全一样，只是命令名不同。

from setuptools import find_packages, setup

package_name = 'f407_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kblue',
    maintainer_email='kblueseek@users.noreply.github.com',
    description='F407 PiComm 串口桥接',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 命令名 f407_bridge_node -> 调用 f407_bridge.f407_bridge_node 里的 main()
            'f407_bridge_node = f407_bridge.f407_bridge_node:main',
        ],
    },
)
