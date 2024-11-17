from setuptools import find_packages, setup

package_name = 'terp2_arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cgcollins91',
    maintainer_email='cgcollins91@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_state_subscriber = terp2_arm_controller.model_state_subscriber:main',
            'joint_state_subscriber = terp2_arm_controller.joint_state_subscriber:main',
            'joint_state_publisher  = terp2_arm_controller.joint_state_publisher:main',
        ],
    },
)
