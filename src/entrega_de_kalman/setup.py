from setuptools import find_packages, setup

package_name = 'entrega_de_kalman'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2d2 = entrega_de_kalman.r2d2:main',
            'r2d2_get_wheel_angles_test = entrega_de_kalman.r2d2_get_wheel_angles_test:main',
        ],
    },
)
