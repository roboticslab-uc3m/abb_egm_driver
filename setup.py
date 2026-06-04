import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'abb_egm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'ABBRobotEGM'],
    zip_safe=True,
    maintainer='Bartek Łukawski',
    maintainer_email='bwmn.peter@gmail.com',
    description='EGM driver for ABB robots',
    license='LGPL v2.1',
    entry_points={
        'console_scripts': [
            'egm_driver = abb_egm_driver.egm_driver:main',
            'keyboard_teleop = abb_egm_driver.keyboard_teleop:main',
        ],
    },
)
