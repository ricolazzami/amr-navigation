from setuptools import setup

package_name = "amr_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ricou",
    maintainer_email="ricou@example.com",
    description="Custom 4-way Manhattan A* planner for the DecodeLabs AMR project",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "a_star_node = amr_planning.a_star_node:main",
        ],
    },
)
