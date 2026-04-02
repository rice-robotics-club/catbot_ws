import os
from setuptools import find_packages, setup
from glob import glob


package_name = 'catbot_ik'
launch_files = glob(os.path.join('launch', '*launch.[pxy][yma]*'))
executables = glob(os.path.join(package_name, "*_node.py"))


def console_script(filename: str) -> str:
   name = filename[len(package_name)+1:len(filename)-3]
   return f"{name} = {package_name}.{name}:main"


setup(
   name=package_name,
   version='0.0.0',
   packages=find_packages(exclude=['test']),
   data_files=[
       ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
       ("share/" + package_name, ["package.xml"]),
       ("share/" + package_name + "/launch", launch_files),
   ],
   install_requires=['setuptools'],
   zip_safe=True,
   maintainer='YOUR_NAME',
   maintainer_email='YOUR_NAME@todo.todo',
   description='TODO: Package description',
   license='Apache-2.0',
   tests_require=['pytest'],
   entry_points={
       "console_scripts": list(map(console_script, executables)),
   },
)





