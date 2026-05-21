from setuptools import find_packages, setup

package_name = 'simple_goal_publisher'

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
    maintainer='juli',
    maintainer_email='ge96fec@mytum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'simple_goal_publisher = simple_goal_publisher.simple_goal_publisher:main',
        'simple_goal_publisher_panda = simple_goal_publisher.simple_goal_publisher_panda:main'
    ],
    },
)
