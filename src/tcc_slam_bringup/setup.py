from glob import glob
import os

from setuptools import find_packages, setup

package_name = "tcc_slam_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name]
        ),
        (
            "share/" + package_name,
            ["package.xml"]
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml")
        ),
        
        (
            'share/' + package_name + '/launch',
            ['launch/readiness_monitor.py'],
        ),
        (
            os.path.join('share', package_name, 'scripts'),
            [
                'scripts/bootstrap_graphslam.py',
                'scripts/landmark_detector.py',
                'scripts/ekf_map_builder.py',
            ],
        ),
        (
            os.path.join('share', package_name, 'config', 'mrpt'),
            glob('config/mrpt/*.ini') + glob('config/mrpt/*.yaml'),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TCC",
    maintainer_email="erick.cesar@academico.ifpb.edu.br",
    description="Inicialização da simulação do TCC de SLAM",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": []
    },
)
