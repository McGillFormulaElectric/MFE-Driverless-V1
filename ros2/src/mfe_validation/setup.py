from setuptools import setup
setup(
    name='mfe_validation', version='0.1.0', packages=['mfe_validation'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/mfe_validation']),
                ('share/mfe_validation', ['package.xml']),
                ('share/mfe_validation/launch', ['launch/validation.launch.py', 'launch/lidar_only.launch.py'])],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': ['slam_node = mfe_validation.slam_node:main',
                                      'compare_node = mfe_validation.compare_node:main',
                                      'skidpad_planner = mfe_validation.skidpad_planner:main']},
)
