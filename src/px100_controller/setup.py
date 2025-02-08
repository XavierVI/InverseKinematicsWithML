from setuptools import find_packages, setup

package_name = 'px100_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/data_generator.launch.py', 'launch/px100_controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xavier',
    maintainer_email='vigenesisvi@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pytorch_controller = px100_controller.pytorch_controller:main',
            'data_generator = px100_controller.data_generator:main'
        ],
    },
)
