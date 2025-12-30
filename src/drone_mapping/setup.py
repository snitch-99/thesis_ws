import os
from glob import glob
from setuptools import find_packages, setup

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

package_name = 'drone_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models/entities/rock'), glob('models/entities/rock/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kanav',
    maintainer_email='kprashar@asu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'traversability = drone_mapping.traversability:main',
            'mavros_control = drone_mapping.mavros_control:main',
        ],
    },
)
