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
    packages=[package_name, 'drone_utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models/entities/rock'), glob('models/entities/rock/*')),
        (os.path.join('share', package_name, 'models/entities/rock2'), glob('models/entities/rock2/*')),
        (os.path.join('share', package_name, 'models/entities/rock3'), glob('models/entities/rock3/*')),
        (os.path.join('share', package_name, 'models/entities/rock4'), glob('models/entities/rock4/*')),
        (os.path.join('share', package_name, 'models/entities/rock5'), glob('models/entities/rock5/*')),
        (os.path.join('share', package_name, 'models/entities/rock6'), glob('models/entities/rock6/*')),
        (os.path.join('share', package_name, 'models/entities/rock7'), glob('models/entities/rock7/*')),
        (os.path.join('share', package_name, 'models/entities/rock8'), glob('models/entities/rock8/*')),
        (os.path.join('share', package_name, 'models/entities/rock9'), glob('models/entities/rock9/*')),
        (os.path.join('share', package_name, 'models/entities/rock10'), glob('models/entities/rock10/*')),
        (os.path.join('share', package_name, 'models/entities/terrain'), glob('models/entities/terrain/*')),
        (os.path.join('share', package_name, 'config'), glob('drone_utils/*.csv') + glob('drone_utils/*.json')),
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

            'synced_broadcaster = drone_mapping.synced_broadcaster:main',
            'trajectory_gui = drone_utils.trajectory_gui:main',
            'cluster_tracker = drone_mapping.cluster_tracker:main',
            'run_controller  = drone_mapping.run_controller:main',
        ],
    },
)
