from setuptools import find_packages, setup

package_name = 'rob_patroller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/' + package_name + '/models', ['models/yellow_box.sdf']),
        ('share/' + package_name + '/launch', ['launch/spawn_objects.launch.py']),
        ('share/' + package_name + '/launch', ['launch/all_systems.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='len',
    maintainer_email='len@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'click_logger = rob_patroller.click_logger:main',
            'waypoint_follower = rob_patroller.waypoint_follower:main',
            'yellow_object_detector = rob_patroller.yellow_object_detector:main',
        ],
    },
)
