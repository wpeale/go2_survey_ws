import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'go2_survey'
k30_submodule = 'go2_survey/k30'
planning_submodule = 'go2_survey/planning_utils'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name, k30_submodule, planning_submodule],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROS 2 Developer',
    author_email='ros2@ros.com',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    keywords=['foo', 'bar'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='My awesome package.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'co2_survey_node = go2_survey.co2_survey_node:main',
            'flux_survey_logger_node = go2_survey.flux_survey_logger_node:main',
            'begin_survey = go2_survey.begin_survey:main'
        ],
    },
)
