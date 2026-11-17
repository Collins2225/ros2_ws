from setuptools import find_packages, setup

package_name = 'obs_avoid_control'

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
    maintainer='collins',
    maintainer_email='collinsnjoku42@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello_robot = obs_avoid_control.hello_robot:main',
            'sweep_scanner = obs_avoid_control.core.sweep_scanner:main',
            'avoidance_node = obs_avoid_control.core.avoidance_node:main',
            'judge = obs_avoid_control.metrics.judge:main',
        ],
    },
)
