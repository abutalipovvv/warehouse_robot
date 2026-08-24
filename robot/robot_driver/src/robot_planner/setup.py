from setuptools import find_packages, setup


package_name = "robot_planner"
packages = find_packages()


setup(
    name=package_name,
    version="0.0.0",
    packages=packages,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy>=1.24"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="kaisar.abutalipovv@gmail.com",
    description="Route planner and execution logic for the warehouse robot.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "route_node=robot_planner.route_node:main",
        ],
    },
)
