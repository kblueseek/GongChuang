# setup.py：告诉 colcon 怎么安装 joy_control 包。
# 和 f407_bridge 结构完全一样，只是命令名不同。

from setuptools import find_packages, setup

package_name = 'joy_control'

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
    description='USB 手柄控制（订阅 /joy，发布整车控制话题）',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 命令名 joy_control_node -> 调用 joy_control.joy_control_node 里的 main()
            'joy_control_node = joy_control.joy_control_node:main',
        ],
    },
)
