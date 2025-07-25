from glob import glob
from setuptools import setup, find_packages

package_name = 'myrobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
        include=[
            'myrobot', 'myrobot.*',
            'feetech_tuna', 'feetech_tuna.*'
        ]
    ),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pim0ubuntu',
    maintainer_email='pim0ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = myrobot.state_publisher:main',
            'send_move = myrobot.send_move:main',
            'monitor_ui = myrobot.monitor_ui:main',
            'cam_input = myrobot.cam_input:main',
        ],
    },
)
