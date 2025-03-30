from setuptools import find_packages, setup
import os
import glob

package_name = 'icp_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avirupghosh',
    maintainer_email='avirupghosh@todo.todo',
    description='ICP-based initial pose estimation using scan matching',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icp_localization = icp_localization.icp_localization:main',
            'some_script = icp_localization.scripts.some_script:main',  # If you have scripts
        ],
    },
)

