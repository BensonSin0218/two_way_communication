from setuptools import find_packages, setup

import glob
import os

package_name = "two_way_communication"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob.glob("config/*.yaml"))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Benson Sin",
    maintainer_email="sinbenson0218@gmail.com",
    description="Two-way communication for network and ROS2",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"main = {package_name}.main:main"
        ],
    },
)
