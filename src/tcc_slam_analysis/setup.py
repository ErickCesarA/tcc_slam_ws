from setuptools import setup

package_name = 'tcc_slam_analysis'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools', 'numpy', 'pandas'],
    zip_safe=True,
    maintainer='erick',
    maintainer_email='',
    description='Extracao de trajetorias e metricas para experimentos SLAM.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'analisar_experimento = '
            'tcc_slam_analysis.analisar_experimento:main',
	    'ground_truth_odom = tcc_slam_analysis.ground_truth_odom:main',
        ],
    },
)
