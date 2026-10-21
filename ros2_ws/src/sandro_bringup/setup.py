from glob import glob
from setuptools import find_packages, setup

package_name = "sandro_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
        ("share/" + package_name + "/tools", glob("tools/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Giuseppe Festa",
    maintainer_email="giuseppe.festa@example.com",
    description="Sandro deployment and bringup",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_estop = sandro_bringup.joy_estop:main",
            "joy_teleop = sandro_bringup.joy_teleop:main",
            "sim_reset = sandro_bringup.sim_reset:main",
        ],
    },
)
