from setuptools import find_packages, setup

package_name = 'terp2_controller_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            ('share/terp2_controller_py/launch', ['launch/controllers_launch.py']),
            ('share/terp2_controller_py/path', ['path/path.csv']),\
            ('share/terp2_controller_py/path', ['path/goals.csv']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cgcollins91',
    maintainer_email='cgcollins91@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'controller_py        = terp2_controller_py.controller_py:main',
                'model_state_to_odom  = terp2_controller_py.model_state_to_odom:main',
                'explorer             = terp2_controller_py.explorer:main',
                'path_follower  = terp2_controller_py.path_follower:main'
                
        ],
    },
)
