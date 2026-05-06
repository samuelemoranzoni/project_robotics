from glob import glob
from setuptools import setup

package_name = "safe_lane_selection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*launch*")),
        ("share/" + package_name + "/scripts", glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samuele Moranzoni",
    maintainer_email="samuele@example.com",
    description="Safe lane selection demo for RoboMaster/CoppeliaSim.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safe_lane_node = safe_lane_selection.safe_lane_node:main",
        ],
    },
)

