from setuptools import setup

package_name = 'pose_publisher_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'tf_transformations'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Publishes poses from .npz file as PoseArray',
    license='MIT',
    entry_points={
        'console_scripts': [
            'publish_npz_poses = pose_publisher_pkg.publish_npz_poses:main',
        ],
    },
)
