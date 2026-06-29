from glob import glob

from setuptools import find_packages, setup


package_name = "real_robot"
packages = find_packages(exclude=["test"]) + ["robot_grpc_api", "robot_grpc_api.proto"]


setup(
    name=package_name,
    version="0.0.0",
    packages=packages,
    package_dir={
        "robot_grpc_api": "../robot_grpc_api",
        "robot_grpc_api.proto": "../robot_grpc_api/proto",
    },
    package_data={"robot_grpc_api.proto": ["*.proto"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="kaisar.abutalipovv@gmail.com",
    description="ROS 2 driver for AIvison Robokit mobile robots.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_driver=real_robot.driver:main",
            "robot_api_server=robot_grpc_api.ros_server_main:main",
        ],
    },
)
