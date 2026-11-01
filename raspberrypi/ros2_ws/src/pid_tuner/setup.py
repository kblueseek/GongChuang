# setup.py：pid_tuner 是纯 Python 包，和 gimbal_track 结构一致。

from setuptools import find_packages, setup

package_name = 'pid_tuner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pid_tuner.launch.py']),
        ('share/' + package_name + '/config', ['config/pid_tuner.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kblue',
    maintainer_email='kblueseek@users.noreply.github.com',
    description='通用 PID 滑杆调参上位机',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pid_tuner_node = pid_tuner.pid_tuner_node:main',
        ],
    },
)
