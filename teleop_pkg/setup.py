from setuptools import find_packages, setup

package_name = 'teleop_pkg'

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
    maintainer='labiba-ibnat-matin',
    maintainer_email='ibnatmatin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'teleop_pwm_keyboard = teleop_pkg.teleop_pwm_keyboard:main',
        'cmd_pwm_udp_sender = teleop_pkg.cmd_pwm_udp_sender:main',
    ],
},
    
)
