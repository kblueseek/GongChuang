# setup.py：告诉 colcon 怎么安装 vision_node 包。结构参照 ring_fix / motion_planner。

from setuptools import find_packages, setup

package_name = 'vision_node'

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
    description='统一视觉节点：单摄像头、多业务（color_pick / vision_put / calibrate）',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 命令名 vision_node -> 调用 vision_node.node 里的 main()
            'vision_node = vision_node.node:main',
        ],
    },
)
