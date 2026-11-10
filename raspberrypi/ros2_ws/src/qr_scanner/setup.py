# setup.py：告诉 colcon 怎么安装 qr_scanner 包。结构和 ring_fix 完全一致。

from setuptools import find_packages, setup

package_name = 'qr_scanner'

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
    description='扫码任务码节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'qr_scanner_node = qr_scanner.qr_scanner_node:main',
        ],
    },
)
