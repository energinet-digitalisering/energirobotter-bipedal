import os
from glob import glob
from setuptools import setup

package_name = 'bipedal_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicoline L. Thomsen',
    maintainer_email='xnlth@energinet.dk',
    description='Integration of Reachy Bipedal in Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_gz_interface = bipedal_gazebo.fake_gz_interface:main'
        ],
    },
)
