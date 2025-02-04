from setuptools import find_packages, setup
import os
import glob

package_name = 'central_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob.glob(os.path.join("launch", "*.launch.py")))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ms',
    maintainer_email='001208yun@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "server = central_control.server:main",
            "individual = central_control.individual:main",
            "commander = central_control.commander:main",
            "test_with_turtle = central_control.test_with_turtle:main",
            "service_provider = central_control.service_provider:main",
            "dist_turtle_action_server = central_control.dist_turtle_action_server:main",
            "my_multi_thread = central_control.my_multi_thread:main",
            "MainServer = central_control.MainServer:main",
            "udp_receiver = central_control.MainServer:main",
            "img_sender = central_control.img_sender:main",
            "PIDcontroller = central_control.PIDcontroller:main",
        ],
    },
)