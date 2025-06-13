from setuptools import find_packages, setup

package_name = 'fr3_test'

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
    maintainer='qpaig',
    maintainer_email='leokeran@mit.edu',
    description='Test for Isaac Sim ROS2 Bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'talker = fr3_test.talker:main'
        ],
    },
)
