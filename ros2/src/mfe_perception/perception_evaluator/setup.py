from setuptools import setup
package_name = 'perception_evaluator'
setup(
    name=package_name, version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={'console_scripts': [
        'evaluator_node = perception_evaluator.evaluator_node:main',
        'fusion_evaluator_node = perception_evaluator.fusion_evaluator_node:main',
    ]},
)
