from setuptools import setup

package_name = 'iplanner'

setup(
 name=package_name,
 version='1.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Fan Yang',
 maintainer_email='fanyang1@ethz.ch',
 description='A ROS wrapper for Imperative Planner (iPlanner) package',
 license='BSD-3',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'iplanner = iplanner.iplanner_node:main'
     ],
   },
)