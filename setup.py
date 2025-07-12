from setuptools import find_packages, setup

package_name = 'jacobi_robot'

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
    maintainer='Shiyu-Liu',
    maintainer_email='shiyu.liu27@outlook.com',
    description='This package implements inverse kinematics control and simulation of Jacobi Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
