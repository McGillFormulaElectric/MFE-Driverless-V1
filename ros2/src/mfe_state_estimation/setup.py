import os
from glob import glob
from setuptools import setup

package_name = "mfe_state_estimation"

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config/', '*.yaml'))),
    ],
    # This is important as well
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    author='MFE',
    author_email='albert@todo.todo',
    maintainer='MFE',
    maintainer_email='albert@todo.todo',
    keywords=[],
    classifiers=[
        'Intended Audience :: todo',
        'License :: GPLv3',
        'Programming Language :: Python',
        'Topic :: todo',
    ],
    description='todo',
    license='GPLv3',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [],
    },
)