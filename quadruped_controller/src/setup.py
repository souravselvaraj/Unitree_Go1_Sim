from setuptools import setup
import os
from glob import glob

package_name = 'quadruped_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sourav',
    maintainer_email='krssourav@gmail.com',
    description='Quadruped control with torso dance functionality',
    license='MIT',
    entry_points={
        'console_scripts': [
            'torso_dance = quadruped_controller.torso_dance:main',
        ],
    },
)
