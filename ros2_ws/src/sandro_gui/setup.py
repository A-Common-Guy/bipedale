from glob import glob
from setuptools import find_packages, setup

package_name = "sandro_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
        ("share/" + package_name + "/web", glob("web/*.*")),
        ("share/" + package_name + "/web/vendor", glob("web/vendor/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Giuseppe Festa",
    maintainer_email="giuseppe.festa@example.com",
    description="Browser control panel for Sandro",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sandro_gui = sandro_gui.main:main",
        ],
    },
)
