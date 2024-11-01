from setuptools import setup
from glob import glob

package_name = 'c_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch/", glob("launch/*launch*")),
        ('share/' + package_name + "/rviz/", glob("rviz/*")),
        ('share/' + package_name + "/c_world/" , glob('c_world/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'c_turtle = c_turtle.c_turtle:main'
        ],
    },
)