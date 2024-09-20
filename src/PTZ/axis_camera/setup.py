from setuptools import setup
import os
from glob import glob

package_name = 'axis_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intelligent Systems Lab',
    maintainer_email='isl.torvergata@gmail.com',
    description='Axis PTZ camera driver module.',
    license='GNU GPL v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'axis_camera = axis_camera.axis_camera_app:main',
            'axis_camera_gazebo = axis_camera.axis_camera_gazebo:main',
            'saver = axis_camera.saver:main',
            'visualizer = axis_camera.visualizer:main'
        ],
    },
)
