import os
from glob import glob
from setuptools import setup

package_name = "vision_cone_detector"


setup(
    name=package_name,
    version='0.1.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'resource'), ['resource/video.mp4']),
        # (os.path.join('share', package_name, 'resource'), ['resource/best.pt']),
    ],
    # This is important as well
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
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'file_loader = vision_cone_detector.file_loader:main',
            'cone_detection_node = vision_cone_detector.cone_detection_node:main'
        ],
    },
)