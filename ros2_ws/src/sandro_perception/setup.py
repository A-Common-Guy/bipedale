from glob import glob
from setuptools import find_packages, setup

package_name = "sandro_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Giuseppe Festa",
    maintainer_email="giuseppe.festa@example.com",
    description="Depth camera to terrain profile for Sandro",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "height_profile = sandro_perception.node:main",
        ],
    },
)
