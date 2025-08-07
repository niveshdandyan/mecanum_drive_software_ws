from setuptools import find_packages, setup
import glob

package_name = 'custom_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/maps',   glob.glob('maps/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dase-pi-5-01',
    maintainer_email='dase-pi-5-01@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # add your console script entry points here
        ],
    },
)