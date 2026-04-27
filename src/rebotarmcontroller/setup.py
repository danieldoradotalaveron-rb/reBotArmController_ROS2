from glob import glob
from setuptools import find_packages, setup

package_name = "rebotarmcontroller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/examples", glob("examples/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="ROS 2 controller node for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "reBotArmController = rebotarmcontroller.rebotarm_controller:main",
        ],
    },
)
