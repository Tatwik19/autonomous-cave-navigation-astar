from setuptools import find_packages, setup

package_name = 'ras598_assignment_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'map.yaml',
            'cave_filled.png',
            'planning.rviz',
            'grading_scout.py',
            'README.md',
        ]),
        ('share/' + package_name + '/launch', ['planner_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eva',
    maintainer_email='eva@todo.todo',
    description='RAS 598 Assignment 2 Motion Planning',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = ras598_assignment_2.planner_node:main',
            # 'grading_scout = grading_scout:main',
        ],
    },
)