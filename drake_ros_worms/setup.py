from glob import glob
from setuptools import find_packages, setup

package_name = 'drake_ros_worms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trevor',
    maintainer_email='trejohst@mit.edu',
    description='Drake interface for WORMS project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "model_interface = drake_ros_worms.model_interface:main",
        ],
    },
)
