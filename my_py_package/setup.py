import os, glob
from setuptools import find_packages, setup

package_name = 'my_py_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[

        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # add launch.py files
        ('share/' + package_name + '/launch', ['launch/start_py_nodes.launch.py'])

    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehmet_Kahraman',
    maintainer_email='Mehmet_Kahraman@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    # add all python scripts
    entry_points={
        'console_scripts': [
            "publisher_timer_node = my_py_package.publisher_timer_node:main",
            "subscriber_node = my_py_package.subscriber_node:main",
            "service_server_node = my_py_package.service_server_node:main",
            "service_client_node = my_py_package.service_client_node:main",
            "using_parameters_node = my_py_package.using_parameters_node:main",
            "action_server_node = my_py_package.action_server_node:main",
            "action_client_node = my_py_package.action_client_node:main"
        ],
    },
)
