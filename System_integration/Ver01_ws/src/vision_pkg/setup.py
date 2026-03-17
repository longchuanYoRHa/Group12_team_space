from pathlib import Path

from setuptools import setup

package_name = 'vision_pkg'
here = Path(__file__).parent
model_dir = here / package_name / "best_openvino_model"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/best_openvino_model', [str(p) for p in model_dir.glob('*') if p.is_file()]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student04',
    maintainer_email='student04@todo.todo',
    description='Rover Vision Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_vision = vision_pkg.rover_vision_node:main',
            'rover_vision_sim = vision_pkg.rover_vision_sim_node:main',
        ],
    },
)
