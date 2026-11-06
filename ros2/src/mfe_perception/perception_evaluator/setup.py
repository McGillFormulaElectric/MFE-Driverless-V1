import glob
import os

from setuptools import setup

package_name = 'perception_evaluator'
setup(
    name=package_name, version='1.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': [
        'evaluator_node = perception_evaluator.evaluator_node:main',
        'fusion_evaluator_node = perception_evaluator.fusion_evaluator_node:main',
    ]},
)
