# Copyright 2024, All rights reserved.
#
# Licensed under the MIT License.

from setuptools import find_packages, setup

package_name = 'biped_genesis_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='kim.xps12@gmail.com',
    description='Genesis physics simulator bridge node for BSL-Droid',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'genesis_sim_node = biped_genesis_sim.genesis_sim_node:main',
        ],
    },
)
