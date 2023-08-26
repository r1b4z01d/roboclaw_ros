from setuptools import setup

package_name = 'roboclaw_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brad Bazemore',
    maintainer_email='bwbazemore@uga.edu',
    description='ROS 2 Roboclaw Node package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robo_node = roboclaw_node.roboclaw_node:main',
            ],
    },
)
