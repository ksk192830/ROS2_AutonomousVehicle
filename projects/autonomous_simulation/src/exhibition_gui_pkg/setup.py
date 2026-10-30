from glob import glob
from setuptools import find_packages, setup


package_name = 'exhibition_gui_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/assets',
            [
                'assets/steering_wheel.png',
                'assets/ASSET_CREDITS.md',
                'assets/ai_reference_lap.json',
            ],
        ),
        ('share/' + package_name + '/assets/examples', glob('assets/examples/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sg',
    maintainer_email='sg@todo.todo',
    description='Visitor-facing autonomous-driving pipeline dashboard.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'exhibition_dashboard = exhibition_gui_pkg.dashboard:main',
            'exhibition_obstacle_manager = exhibition_gui_pkg.obstacle_manager:main',
            'exhibition_race_game = exhibition_gui_pkg.race_game:main',
            'exhibition_linux_joy = exhibition_gui_pkg.linux_joy:main',
        ],
    },
)
