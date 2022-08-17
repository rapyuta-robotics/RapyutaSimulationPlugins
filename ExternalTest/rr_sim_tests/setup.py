from setuptools import setup

package_name = 'rr_sim_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duc',
    maintainer_email='ducanh.than@rapyuta-robotics.com',
    description='ROS2-based tests for Rapyuta Sim',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_random_spawn = rr_sim_tests.test_random_spawn:main',
        ],
    },
)
