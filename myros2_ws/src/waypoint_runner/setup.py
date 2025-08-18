from setuptools import setup

package_name = 'waypoint_runner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'web3'],
    zip_safe=True,
    maintainer='tersoo',
    maintainer_email='samuel.awai5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visit_points = waypoint_runner.visit_points:main',
        ],
    },
)
