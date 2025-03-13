from setuptools import find_packages, setup

package_name = 'iiwa_pasme'

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
    maintainer='juan',
    maintainer_email='juan.sandoval@ec-nantes.fr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'iiwa_joint_pos_pub = iiwa_pasme.iiwa_joint_pos:main',
        	'teleop_gui_pub = iiwa_pasme.teleop_gui:main',
            'iiwa_markers_pub = iiwa_pasme.iiwa_markers:main', 
            'iiwa_markers_traj_pub = iiwa_pasme.iiwa_markers_traj:main',
        ],
    },
)
