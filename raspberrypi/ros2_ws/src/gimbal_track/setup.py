# setup.py：告诉 colcon 怎么安装 gimbal_track 包（和 joy_control 结构一致）。

from setuptools import find_packages, setup

package_name = 'gimbal_track'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gimbal_track.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kblue',
    maintainer_email='kblueseek@users.noreply.github.com',
    description='视觉跟随云台 demo：物料识别 + 像素误差 PID 控制 yaw/extend',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 命令名 -> 模块里的 main() 函数
            'material_detect_node = gimbal_track.material_detect_node:main',
            'gimbal_track_node = gimbal_track.gimbal_track_node:main',
        ],
    },
)
