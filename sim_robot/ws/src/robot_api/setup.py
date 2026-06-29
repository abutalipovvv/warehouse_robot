from setuptools import setup


package_name = "robot_api"


setup(
    name=package_name,
    version="0.0.0",
    packages=["robot_api", "robot_grpc_api", "robot_grpc_api.proto"],
    package_dir={
        "robot_grpc_api": "../robot_grpc_api",
        "robot_grpc_api.proto": "../robot_grpc_api/proto",
    },
    package_data={"robot_grpc_api.proto": ["*.proto"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="kaisar.abutalipovv@gmail.com",
    description="Native gRPC robot API backed by local ROS 2 topics and services.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_api_server=robot_grpc_api.ros_server_main:main",
        ],
    },
)
