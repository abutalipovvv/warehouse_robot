from setuptools import find_packages, setup


package_name = "robot_motion_gateway"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="warehouse_robot",
    maintainer_email="kaisar.abutalipovv@gmail.com",
    description="Single-owner velocity command gateway for the warehouse robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_gateway=robot_motion_gateway.node:main",
        ],
    },
)
