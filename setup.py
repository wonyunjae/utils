from setuptools import find_packages, setup

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/' + package_name + '/launch', ['launch/odom_to_utm.launch.py']),
    ],
    install_requires=['setuptools'],
    extras_require={'filterpy': ['filterpy']},
    zip_safe=True,
    maintainer='wynz',
    maintainer_email='wyj5578@koreatech.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_converter = utils.map_converter:main',
            'imu_filter = utils.imu_filter:main',
            'rel_to_abs = utils.rel_to_abs:main',
            'gps_to_utm = utils.gps_to_utm:main',
            'odom_to_utm_point = utils.odom_to_utm_point:main',
            'save_tum = utils.save_tum:main',
        ],
    },
)
