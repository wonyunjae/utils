from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
glob(os.path.join('launch', '*launch.[pxy][ymal]*'))),
    ],
    install_requires=['setuptools'],
        zip_safe=True,
    maintainer='wynz',
    maintainer_email='wyj5578@koreatech.ac.kr',
    description='Utility package for localization system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [    
            'map_converter = utils.map_converter:main',
            'imu_filter = utils.imu_filter:main',
            'rel_to_abs = utils.rel_to_abs:main',
            'gps_to_utm = utils.gps_to_utm:main',
            'odom_to_utm_point = utils.odom_to_utm_point:main',
            'save_tum = utils.save_tum:main',
            'camera_image_saver = utils.camera_image_saver:main',
            'ply_publisher = utils.ply_publisher:main',
            'localization_ui = utils.localization_ui:main',
        ],
    },
)
