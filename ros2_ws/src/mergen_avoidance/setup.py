from glob import glob
import os

from setuptools import find_packages, setup

package_name = "mergen_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mergen-01",
    maintainer_email="204471282+oguzhanhekimoglu@users.noreply.github.com",
    description="TEKNOFEST Robotaksi: lidar tabanli otonom engelden sakinma (TurtleBot3/Gazebo)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"avoider = {package_name}.avoider_node:main",
            f"metrics_logger = {package_name}.metrics_logger:main",
            f"pedestrian = {package_name}.pedestrian_node:main",
        ],
    },
)
