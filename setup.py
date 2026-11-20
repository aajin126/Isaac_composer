import os

from setuptools import find_packages, setup


PACKAGE_NAME = "isaac_composer"


def package_data_files():
    entries = []
    for root in ("config", "docs", "launch", "maps", "params", "rviz", "scripts", "usd", "third_party"):
        for directory, subdirectories, filenames in os.walk(root):
            subdirectories[:] = [
                name
                for name in subdirectories
                if not name.startswith((".", "__"))
            ]
            files = [
                os.path.join(directory, filename)
                for filename in filenames
                if not filename.startswith(".") and not filename.endswith(".pyc")
                and os.path.isfile(os.path.join(directory, filename))
            ]
            if files:
                entries.append((os.path.join("share", PACKAGE_NAME, directory), files))
    return entries


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        *package_data_files(),
    ],
    install_requires=["setuptools", "PyYAML>=6.0", "Pillow", "numpy", "scipy"],
    zip_safe=True,
    maintainer="ewhaglab",
    maintainer_email="maintainer@example.com",
    description="Configuration-driven Isaac Sim robot and scenario composition",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "isaac_composer_validate = isaac_composer.cli:main",
            "isaac_composer_sim = isaac_composer.runtime:main",
            "lidar_fusion_node = isaac_composer.sensors.lidar_fusion_node:main",
            "laser_scan_fusion_node = isaac_composer.sensors.laser_scan_fusion_node:main",
        ],
    },
)
