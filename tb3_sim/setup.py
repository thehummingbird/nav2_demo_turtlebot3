from setuptools import setup
import os
from glob import glob

package_name = 'tb3_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name),
         glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config/'),
         glob('config/*')),
        (os.path.join('share', package_name, 'models/green_block/'),
         glob('models/green_block/*')),
        (os.path.join('share', package_name, 'models/blue_block/'),
         glob('models/blue_block/*')),
        (os.path.join('share', package_name, 'models/red_block/'),
         glob('models/red_block/*')),
        (os.path.join('share', package_name, 'maps/'),
         glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sharad',
    maintainer_email='sharadmaheshwari19@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'block_spawner = tb3_sim.block_spawner:main',
            'amcl_init_pose_publisher = tb3_sim.set_init_amcl_pose:main',
        ],
    },
)
