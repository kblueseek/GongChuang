# setup.py：告诉 colcon 怎么安装 ring_fix 包。结构和 motion_planner 完全一致。

from setuptools import find_packages, setup

package_name = 'ring_fix'

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
    description='圆环工位视觉对位 + 里程计重置',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 命令名 ring_fix_node -> 调用 ring_fix.ring_fix_node 里的 main()
            'ring_fix_node = ring_fix.ring_fix_node:main',
        ],
    },
)
