from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'observer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), #Automatically install nested packages that are in observer. You can install them with import package_name.folders or 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        ('share/' + package_name, glob('**/*.so', recursive=True)), #look the .so in every subfolder

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # Install world files
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*')),

        (os.path.join('share', package_name, 'models'),
         glob('models/*')),

        # (os.path.join('share', package_name, 'matlab'),
        #  glob('matlab/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'observer = observer.observer:main',
        ],
    }
)
