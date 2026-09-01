from setuptools import find_packages, setup
import os
from glob import glob



package_name = 'mjcf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

                # Install world files
        (os.path.join('share', package_name, 'models'),
         glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='CChiariello@unibz.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'converter2mujoco = mjcf.convert_urdf_to_mj:main',
        ],
    },
)
