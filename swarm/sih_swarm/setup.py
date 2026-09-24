import os
from glob import glob
from setuptools import setup, find_packages

package_name = "sih_swarm"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name] if os.path.exists("resource/" + package_name) else []),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SIH26177 Team",
    maintainer_email="team@sih26177.org",
    description="Multi-UAV swarm management for SIH26177.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "swarm_manager_node = sih_swarm.swarm_manager_node:main",
        ],
    },
)
