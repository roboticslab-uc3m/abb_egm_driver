from setuptools import find_packages, setup

package_name = 'abb_egm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antonio',
    maintainer_email='antonio@todo.todo',
    description='Driver EGM para ABB Robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # AQUÍ ESTÁ EL CAMBIO IMPORTANTE:
            'egm_driver = abb_egm_driver.egm_driver:main',
            'keyboard_teleop = abb_egm_driver.keyboard_teleop:main', # <--- NUEVA LÍNEA
        ],
    },
)
