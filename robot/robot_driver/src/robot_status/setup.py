from setuptools import find_packages, setup


package_name = "robot_status"


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
    description="Robot status aggregation node for localization, route execution, and manual control.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_bms_publisher=robot_status.fake_bms:main",
            "status_node=robot_status.main:main",
        ],
    },
)
