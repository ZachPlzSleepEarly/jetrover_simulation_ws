import os
from glob import glob
from setuptools import setup
from typing import Final

PACKAGE_NAME: Final = 'jetrover_description'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', PACKAGE_NAME, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', PACKAGE_NAME, 'urdf/acker'), glob(os.path.join('urdf/acker', '*.*'))),
        (os.path.join('share', PACKAGE_NAME, 'rviz'), glob(os.path.join('rviz', '*.*'))),
        (os.path.join('share', PACKAGE_NAME, 'meshes/acker'), glob(os.path.join('meshes/acker', '*.*'))),
        (os.path.join('share', PACKAGE_NAME, 'meshes/arm'), glob(os.path.join('meshes/arm', '*.*'))),
        (os.path.join('share', PACKAGE_NAME, 'meshes/common'), glob(os.path.join('meshes/common', '*.*'))),
        (os.path.join('share', PACKAGE_NAME, 'meshes/gripper'), glob(os.path.join('meshes/gripper', '*.*'))),
        (os.path.join('share', PACKAGE_NAME, 'scripts'), glob(os.path.join('scripts', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
    scripts=[
        'scripts/cmd_vel_to_ackermann_ref.py',
    ],
)
