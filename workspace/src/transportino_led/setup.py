from setuptools import setup

package_name = 'transportino_led'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MattSays',
    maintainer_email='transportino@mattsays.tech',
    description='Status led controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_ctrl_node = transportino_led.transportino:main'
        ],
    },
)
