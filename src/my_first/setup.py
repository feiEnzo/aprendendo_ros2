from setuptools import find_packages, setup

package_name = 'my_first'

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
    maintainer='Enzo',
    maintainer_email='unieeguillaumon@fei.edu.br',
    description='Meu primeiro pacote',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_one = my_first.first_node:main'
        ],
    },
)
