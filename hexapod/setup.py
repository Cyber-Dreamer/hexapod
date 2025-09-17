import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hexapod'

def get_files(pattern):
    return [f for f in glob(pattern, recursive=True) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), get_files('launch/*')),
        (os.path.join('share', package_name, 'urdf'), get_files('urdf/*')),
        (os.path.join('share', package_name, 'urdf', 'meshes'), get_files('urdf/meshes/*')),
        (os.path.join('share', package_name, 'rviz'), get_files('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyberdreamer',
    maintainer_email='gata23laroche@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hexapod_joint_slider = hexapod.joint_slider:main',
        ],
    },
)
