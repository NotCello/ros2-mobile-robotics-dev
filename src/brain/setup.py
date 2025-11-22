from setuptools import setup
import os
from glob import glob

package_name = 'brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ensure launch files are installed
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='utente',
    maintainer_email='user@todo.todo',
    description='Logic package for assignment',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logic_node = brain.logic_controller:main',
            
        ],
    },
)