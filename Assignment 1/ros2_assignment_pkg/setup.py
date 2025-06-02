from setuptools import setup

package_name = 'ros2_assignment_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shreyans-jain',
    maintainer_email='shreyans-jain@example.com',
    description='ROS2 Assignment Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_filter_node = ros2_assignment_pkg.pose_filter_node:main',
            'transform_service_node = ros2_assignment_pkg.transform_service_node:main',
            'test_publisher = ros2_assignment_pkg.test_publisher:main',
            'test_client = ros2_assignment_pkg.test_client:main',

        ],
    },
)