from setuptools import find_packages, setup

package_name = 'terp2_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/terp2_slam/launch', ['launch/slam_custom.launch.py']),
        ('share/terp2_slam/rviz', ['rviz/slam_custom.rviz']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cgcollins91',
    maintainer_email='cgcollins91@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapper_node   = terp2_slam.mapper_node:main',
            'scan_matcher  = terp2_slam.scan_matcher:main',
            'box_filter    = terp2_slam.box_filter:main',
            "save_grid     = terp2_slam.save_grid:main",
        ],
    },
)
