from setuptools import setup

package_name = 'cmd_vel_to_stm32'

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
    maintainer='slamrobot',
    maintainer_email='slamrobot@example.com',
    description='Bridge cmd_vel Twist messages to STM32 serial commands',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = cmd_vel_to_stm32.cmd_vel_bridge:main',
        ],
    },
)
