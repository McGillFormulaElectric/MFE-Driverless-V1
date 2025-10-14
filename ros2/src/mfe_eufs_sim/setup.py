import glob
import os
from setuptools import setup

package_name = 'mfe_eufs_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # ament index resource
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='MFE',
    author_email='lauren@todo.todo',
    maintainer='MFE',
    maintainer_email='lauren@todo.todo',
    keywords=[],
    classifiers=[
        'Intended Audience :: todo',
        'License :: GPLv3',
        'Programming Language :: Python',
        'Topic :: todo',
    ],
    description='todo',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'bridge_node = mfe_eufs_sim.bridge_node:main',
        ],
    },
)
