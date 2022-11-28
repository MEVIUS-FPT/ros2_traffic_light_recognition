import os
from glob import glob
from setuptools import setup

package_name = "traffic_light_recognition_fpga_node"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/" + package_name,
            ["package.xml", package_name + "/io_maps.json"],
        ),
        ("share/" + package_name, glob("launch/*_launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fpga_node = traffic_light_recognition_fpga_node.fpga_node:main",
            "talker = traffic_light_recognition_fpga_node.talker:main",
            "listener = traffic_light_recognition_fpga_node.listener:main",
            "draw_circle = traffic_light_recognition_fpga_node.recognition_result_listener:main",
        ],
    },
)
