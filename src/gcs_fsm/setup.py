import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gcs_fsm'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intelligent Systems Lab',
    maintainer_email='isl.torvergata@gmail.com',
    description='GCS finite-state machine.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gcs_fsm = gcs_fsm.gcs_fsm_app:main'
        ],
    },
)
