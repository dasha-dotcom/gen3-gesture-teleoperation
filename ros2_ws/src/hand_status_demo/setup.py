import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hand_status_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dasha',
    maintainer_email='dasha@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'hand_status_publisher = hand_status_demo.hand_status_publisher:main',
		'hand_status_subscriber = hand_status_demo.hand_status_subscriber:main',
		'hand_position_publisher = hand_status_demo.hand_position_publisher:main',
		'hand_position_subscriber = hand_status_demo.hand_position_subscriber:main',
    	],
    },
)
