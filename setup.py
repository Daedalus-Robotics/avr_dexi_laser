from setuptools import find_packages, setup

package_name = "avr_dexi_laser"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nobuharu Shimazu",
    maintainer_email="nobu.bichanna@gmail.com",
    description="A node to fire laser",
    license="LGPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["laser_node = avr_dexi_laser.laser:main"],
    },
)
