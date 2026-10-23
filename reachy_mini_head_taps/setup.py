from glob import glob

from setuptools import find_packages, setup

package_name = 'reachy_mini_head_taps'

setup(
    name=package_name,
    version='1.7.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    extras_require={'test': ['pytest']},   # tells colcon to run the tests with pytest
    maintainer='Séverin Lemaignan',
    maintainer_email='severin.lemaignan@iiia.csic.es',
    description='Wake the Reachy Mini up, or put it to sleep, by tapping on its head.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'head_taps = reachy_mini_head_taps.head_taps:main',
        ],
    },
)
