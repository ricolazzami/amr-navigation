from glob import glob

from setuptools import setup

package_name = "amr_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/maps", glob("maps/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ricou",
    maintainer_email="ricou@example.com",
    description="Launch files, Gazebo world and parameters for the AMR project",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obstacle_mover = amr_bringup.obstacle_mover:main",
            "odom_frame_relay = amr_bringup.odom_frame_relay:main",
        ],
    },
)
