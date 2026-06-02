from setuptools import find_packages, setup


package_name = "robot_gateway"


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
    description="ROS 2 gateway node between robot services/status topics and external transports.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_gateway_node=robot_gateway.gateway_node:main",
        ],
    },
)
