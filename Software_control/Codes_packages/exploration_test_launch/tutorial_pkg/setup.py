from setuptools import find_packages, setup

package_name = 'tutorial_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/explore.yaml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py', 'launch/explore.launch.py', 'launch/localization.launch.py', 'launch/tb3_explore.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student21',
    maintainer_email='2674312287@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
