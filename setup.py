from setuptools import find_packages, setup

package_name = "iso21423_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/connector.launch.py"]),
        ("share/" + package_name + "/config", ["config/nav2_default.yaml", "config/example_vendor_hook.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenRobOps",
    maintainer_email="dev@openrobops.org",
    description="ROS 2 to ISO 21423 connector",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["iso21423_connector = iso21423_ros.node:main"]},
)
