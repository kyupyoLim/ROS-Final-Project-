from setuptools import find_packages
from setuptools import setup

package_name = 'nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gnd0',
    maintainer_email='greattoe@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'follow_waypoint = nav_pkg.script.follow_waypoint:main',
                'follow_waypoint1 = nav_pkg.script.follow_waypoint1:main',
                'follow_waypoint2 = nav_pkg.script.follow_waypoint2:main',
                'follow_waypoint3 = nav_pkg.script.follow_waypoint3:main',
                'follow_waypoint4 = nav_pkg.script.follow_waypoint4:main',
                'pub_goal_msg = nav_pkg.script.pub_goal_msg:main',
                'amcl_pose = nav_pkg.script.amcl_pose:main',
                'Final_nav = nav_pkg.script.Final_nav:main',
                'start_pub = nav_pkg.script.start_pub:main',
        ],
    },
)
