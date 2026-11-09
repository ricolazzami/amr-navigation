from glob import glob

from setuptools import setup

package_name = "amr_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ricou",
    maintainer_email="ricou@example.com",
    description="URDF/xacro and Gazebo sensor configuration for the AMR robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
