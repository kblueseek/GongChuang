from glob import glob
import os

from setuptools import setup

package_name = "vision_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        # ament 索引标记，ros2 run / launch 靠它找到包
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            ["resource/" + package_name],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kblue",
    maintainer_email="kblueseek@users.noreply.github.com",
    description="视觉节点：两路相机 → 识别结果话题",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vision_node = vision_node.vision_node:main",
        ],
    },
)
