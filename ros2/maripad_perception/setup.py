from glob import glob
from setuptools import find_packages, setup

package_name = "maripad_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/calibration", glob("calibration/*")),
        (f"share/{package_name}/notebooks", glob("notebooks/*")),
    ],
    install_requires=["setuptools", "numpy", "opencv-python", "pyyaml"],
    zip_safe=False,
    maintainer="MARIPAD Developer",
    maintainer_email="user@example.com",
    description="Classical RGB-D book pose detection for MARIPAD.",
    license="MIT",
    tests_require=["pytest"],
    scripts=[
        "scripts/capture_book_frames.py",
        "scripts/visualize.py",
        "scripts/evaluate.py",
    ],
)
