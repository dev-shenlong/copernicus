from setuptools import find_packages, setup

package_name = 'mesh_nav_cmd_cvt'

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
    maintainer='shenlong',
    maintainer_email='dev.shenlong@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stmp2uns = mesh_nav_cmd_cvt.stmp2uns:main',
        ],
    },
)
