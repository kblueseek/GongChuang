from setuptools import find_packages, setup

package_name = 'obstacle_detect'

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
    description='扇区障碍检测',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_detect_node = obstacle_detect.obstacle_detect_node:main',
        ],
    },
)
