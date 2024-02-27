from setuptools import find_packages, setup

package_name = 'battery_pkg'

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
    maintainer='raf',
    maintainer_email='raf@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "battery_state_node= battery_pkg.battery_state_node:main",
            "led_panel_node= battery_pkg.led_panel_node:main"

        ],
    },
)
