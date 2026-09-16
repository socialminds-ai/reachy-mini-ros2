from glob import glob

from setuptools import find_packages, setup

package_name = 'reachy_mini_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Séverin Lemaignan',
    maintainer_email='severin.lemaignan@iiia.csic.es',
    description='Bare-metal hardware driver for the Reachy Mini (Python).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'driver_node = reachy_mini_driver.driver_node:main',
        ],
    },
)
