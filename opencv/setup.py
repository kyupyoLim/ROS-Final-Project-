from setuptools import find_packages
from setuptools import setup

package_name = 'opencv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ground0',
    maintainer_email='ground0@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grayscale      = opencv.script.grayscale:main',
            'FindBall      = opencv.script.FindBall:main',
            'FindBall2      = opencv.script.FindBall2:main',
            'get_blue     = opencv.script.get_blue:main',
            'pub_tb3_pose2d      = opencv.script.pub_tb3_pose2d:main',
            'Final_FindBall      = opencv.script.Final_FindBall:main',
            'Last_FindBall = opencv.script.Last_FindBall:main',
            'Last_FindBall_nav = opencv.script.Last_FindBall_nav:main',
            'Default_nav = opencv.script.Default_nav:main',
            'Serial_msg = opencv.script.Serial_msg:main',
        ],
    },
)
