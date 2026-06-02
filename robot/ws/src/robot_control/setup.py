from setuptools import find_packages, setup


package_name = "robot_control"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="kaisar.abutalipovv@gmail.com",
    description="ROS 2 command services for teleop, route execution, cancel, and stop.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_control_node=robot_control.command_node:main",
        ],
    },
)
